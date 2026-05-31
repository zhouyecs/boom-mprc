package boom.v3.ifu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.{Field, Parameters}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._

import boom.v3.common._
import boom.v3.util.{BoomCoreStringPrefix, WrapInc}
import scala.math.{min, max, round, pow, floor}


case class BoomGEHLParams(
  nTables:    Int = 8,
  tableSize:  Int = 256,
  weightBits: Int = 7,
  maxHist:    Int = 128,
  minHist:    Int = 4
) {
  require(nTables >= 2)
  require(isPow2(tableSize))

  // geometric history lengths: table 0 = bias (len 0), tables 1..nTables-1 geometric
  def histLengths: Seq[Int] = {
    0 +: (1 until nTables).map { i =>
      val t = (i - 1).toDouble / (nTables - 2).toDouble
      round(minHist * pow(maxHist.toDouble / minHist.toDouble, t)).toInt
    }
  }

  // Jimenez static threshold
  def theta: Int = floor(1.93 * maxHist).toInt + 14
}


// shared indices, per-bank-slot sums
class GEHLMeta(val nTables: Int, val nIdxBits: Int, val sumBits: Int)(implicit p: Parameters) extends BoomBundle()(p)
  with HasBoomFrontendParameters
{
  val sums = Vec(bankWidth, SInt(sumBits.W))
  val idxs = Vec(nTables, UInt(nIdxBits.W))
}


class GEHLBranchPredictorBank(params: BoomGEHLParams = BoomGEHLParams())(implicit p: Parameters) extends BranchPredictorBank()(p)
{
  val nTables   = params.nTables
  val tableSize = params.tableSize
  val weightBits = params.weightBits
  val nIdxBits  = log2Ceil(tableSize)
  val histLens  = params.histLengths
  val theta     = params.theta

  // sum width: nTables × max_weight
  val sumBits = log2Ceil(nTables * ((1 << (weightBits - 1)) - 1) + 1) + 1
  val maxWeight = ((1 << (weightBits - 1)) - 1).S(weightBits.W)
  val minWeight = (-(1 << (weightBits - 1))).S(weightBits.W)

  require(histLens.last <= globalHistoryLength)

  // ---- folded history (combinational, same logic as TAGE's compute_folded_hist) ----
  def foldHist(hist: UInt, histLen: Int, foldW: Int): UInt = {
    if (histLen == 0) {
      0.U(foldW.W)
    } else {
      val nChunks = (histLen + foldW - 1) / foldW
      val chunks = (0 until nChunks).map { i =>
        hist(min((i + 1) * foldW, histLen) - 1, i * foldW)
      }
      chunks.reduce(_ ^ _)
    }
  }

  // ---- weight tables (Reg for combinational read, simple single-cycle write) ----
  val tables = Seq.tabulate(nTables) { t =>
    RegInit(VecInit(Seq.fill(tableSize)(VecInit(Seq.fill(bankWidth)(0.S(weightBits.W))))))
  }

  val mems = (0 until nTables).map { t =>
    (s"gehl_t$t", tableSize, bankWidth * weightBits)
  }

  // ---- meta ----
  override val metaSz = bankWidth * sumBits + nTables * nIdxBits
  require(metaSz <= bpdMaxMetaLength)

  // ---- reset ----
  val doing_reset = RegInit(true.B)
  val reset_idx = RegInit(0.U(nIdxBits.W))
  reset_idx := reset_idx + doing_reset
  when (reset_idx === (tableSize - 1).U) { doing_reset := false.B }
  when (doing_reset) {
    for (t <- 0 until nTables) {
      tables(t)(reset_idx) := VecInit(Seq.fill(bankWidth)(0.S(weightBits.W)))
    }
  }

  // ============================================================
  // Prediction path
  //
  // F0: PC hash  →  s1_pc_hash
  // F1: ghist arrives → fold history → index = PC_hash XOR fold → Reg read (combinational)
  // F2: sum → prediction → io.resp.f2
  // F3: register prediction, capture meta (sums + shared indices)
  // ============================================================

  val s1_pc_hash = RegNext(fetchIdx(io.f0_pc))

  // F1: compute indices
  val s1_idxs = Wire(Vec(nTables, UInt(nIdxBits.W)))
  for (t <- 0 until nTables) {
    if (t == 0) {
      s1_idxs(t) := s1_pc_hash(nIdxBits - 1, 0)
    } else {
      val folded = foldHist(io.f1_ghist, histLens(t), nIdxBits)
      s1_idxs(t) := (s1_pc_hash ^ folded)(nIdxBits - 1, 0)
    }
  }

  // combinational read from Reg tables
  val s1_weights = Wire(Vec(nTables, Vec(bankWidth, SInt(weightBits.W))))
  for (t <- 0 until nTables; w <- 0 until bankWidth) {
    s1_weights(t)(w) := tables(t)(s1_idxs(t))(w)
  }

  // F2: sum + prediction
  val s2_resp = Wire(Vec(bankWidth, Bool()))
  val s2_sum  = Wire(Vec(bankWidth, SInt(sumBits.W)))
  val s2_idxs = RegNext(s1_idxs)

  for (w <- 0 until bankWidth) {
    val terms: Seq[SInt] = (0 until nTables).map { t => RegNext(s1_weights(t)(w)) }
    def adderTree(xs: Seq[SInt]): SInt = {
      require(xs.nonEmpty)
      if (xs.length == 1) xs.head
      else adderTree(xs.grouped(2).map {
        case a +: b +: _ => a +& b
        case a +: _      => a
      }.toSeq)
    }
    s2_sum(w)  := adderTree(terms)
    s2_resp(w) := s2_valid && s2_sum(w) >= 0.S && !doing_reset

    io.resp.f2(w).taken := s2_resp(w)
    io.resp.f3(w).taken := RegNext(s2_resp(w))
  }

  // F3: meta (shared indices, per-slot sums)
  val s3_sum  = RegNext(s2_sum)
  val s3_idxs = RegNext(s2_idxs)

  val s3_meta_wire = Wire(new GEHLMeta(nTables, nIdxBits, sumBits))
  s3_meta_wire.sums := s3_sum
  s3_meta_wire.idxs := s3_idxs
  io.f3_meta := s3_meta_wire.asUInt

  // ============================================================
  // Update path
  //
  // s1_update arrives with meta (sums + indices from prediction time)
  // Train all tables if |sum| < theta OR the branch was mispredicted.
  // ============================================================
  val s1_update_meta = s1_update.bits.meta.asTypeOf(new GEHLMeta(nTables, nIdxBits, sumBits))
  val s1_gehl_update_valid = !doing_reset && s1_update.valid && s1_update.bits.is_commit_update

  for (w <- 0 until bankWidth) {
    when (s1_gehl_update_valid && s1_update.bits.br_mask(w)) {
      val was_taken = s1_update.bits.cfi_idx.valid &&
        (s1_update.bits.cfi_idx.bits === w.U) && s1_update.bits.cfi_taken
      val mispred   = s1_update.bits.cfi_idx.valid &&
        (s1_update.bits.cfi_idx.bits === w.U) && s1_update.bits.cfi_mispredicted

      val stored_sum = s1_update_meta.sums(w)
      val abs_sum    = Mux(stored_sum >= 0.S, stored_sum, -stored_sum)
      val train      = mispred || abs_sum < theta.S

      when (train) {
        for (t <- 0 until nTables) {
          val idx   = s1_update_meta.idxs(t)
          val old_w = tables(t)(idx)(w)
          val inc   = was_taken
          val new_w = Mux(inc,
            Mux(old_w === maxWeight, maxWeight, old_w + 1.S),
            Mux(old_w === minWeight, minWeight, old_w - 1.S))
          tables(t)(idx)(w) := new_w
        }
      }
    }
  }
}
