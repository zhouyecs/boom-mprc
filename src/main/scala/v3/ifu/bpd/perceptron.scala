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

  // geometric history lengths: table 0 = bias (len 0), tables 1..n-1 geometric minHist..maxHist
  def histLengths: Seq[Int] = {
    0 +: (1 until nTables).map { i =>
      val t = (i - 1).toDouble / (nTables - 2).toDouble
      round(minHist * pow(maxHist.toDouble / minHist.toDouble, t)).toInt
    }
  }

  // Jimenez static threshold (TODO: make adaptive later)
  def theta: Int = floor(1.93 * maxHist).toInt + 14
}


// carried from predict to update: per-slot sums + shared per-table indices
class GEHLMeta(val nTables: Int, val nIdxBits: Int, val sumBits: Int)(implicit p: Parameters) extends BoomBundle()(p)
  with HasBoomFrontendParameters
{
  val sums = Vec(bankWidth, SInt(sumBits.W))
  val idxs = Vec(nTables, UInt(nIdxBits.W))
}


class GEHLBranchPredictorBank(params: BoomGEHLParams = BoomGEHLParams())(implicit p: Parameters) extends BranchPredictorBank()(p)
{
  val nTables    = params.nTables
  val tableSize  = params.tableSize
  val weightBits = params.weightBits
  val nIdxBits   = log2Ceil(tableSize)
  val histLens   = params.histLengths
  val theta      = params.theta

  // sum width: nTables * max_weight, plus sign
  val sumBits   = log2Ceil(nTables * ((1 << (weightBits - 1)) - 1) + 1) + 1
  val maxWeight = ((1 << (weightBits - 1)) - 1).S(weightBits.W)
  val minWeight = (-(1 << (weightBits - 1))).S(weightBits.W)

  require(histLens.last <= globalHistoryLength,
    s"GEHL maxHist ${histLens.last} exceeds globalHistoryLength $globalHistoryLength")

  // ---- folded history (combinational), same idea as TAGE's fold ----
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

  // ---- weight tables: SyncReadMem so they map to BRAM (NOT LUT register files) ----
  // Each entry holds bankWidth weights; a masked write updates a single slot.
  val tables = Seq.fill(nTables)(SyncReadMem(tableSize, Vec(bankWidth, SInt(weightBits.W))))

  // for BOOM SRAM reporting
  val mems = (0 until nTables).map { t => (s"gehl_t$t", tableSize, bankWidth * weightBits) }

  override val metaSz = bankWidth * sumBits + nTables * nIdxBits
  require(metaSz <= bpdMaxMetaLength)

  // ---- reset walk: zero the SRAMs (SyncReadMem cannot use RegInit) ----
  val doing_reset = RegInit(true.B)
  val reset_idx   = RegInit(0.U(nIdxBits.W))
  reset_idx := reset_idx + doing_reset
  when (reset_idx === (tableSize - 1).U) { doing_reset := false.B }

  // ============================================================
  // Prediction path
  //   F1: idx = PC_hash XOR folded(ghist) ; issue SyncReadMem read
  //   F2: read data lands (SRAM's own output reg) -> sum -> resp.f2
  //   F3: register resp -> resp.f3 ; capture meta
  // NOTE: the SyncReadMem 1-cycle read latency REPLACES the old RegNext on
  //       the weights. Read is issued in F1 (address available), data is used
  //       in F2. Do not add a RegNext on the read data.
  // ============================================================
  val s1_pc_hash = RegNext(fetchIdx(io.f0_pc))

  val s1_idxs = Wire(Vec(nTables, UInt(nIdxBits.W)))
  for (t <- 0 until nTables) {
    if (t == 0) {
      s1_idxs(t) := s1_pc_hash(nIdxBits - 1, 0)
    } else {
      val folded = foldHist(io.f1_ghist, histLens(t), nIdxBits)
      s1_idxs(t) := (s1_pc_hash ^ folded)(nIdxBits - 1, 0)
    }
  }

  // F1 read -> data valid in F2.  (s1_valid is the F1-stage valid from the base
  // class; if unavailable in your tree, true.B is functionally fine for BRAM.)
  val s2_weights = VecInit((0 until nTables).map { t =>
    tables(t).read(s1_idxs(t), s1_valid)
  })

  val s2_sum  = Wire(Vec(bankWidth, SInt(sumBits.W)))
  val s2_resp = Wire(Vec(bankWidth, Bool()))
  val s2_idxs = RegNext(s1_idxs)

  for (w <- 0 until bankWidth) {
    val terms = VecInit((0 until nTables).map { t => s2_weights(t)(w) })
    s2_sum(w)  := terms.reduceTree(_ +& _)
    s2_resp(w) := s2_valid && (s2_sum(w) >= 0.S) && !doing_reset
    io.resp.f2(w).taken := s2_resp(w)
    io.resp.f3(w).taken := RegNext(s2_resp(w))
  }

  // F3 meta (shared indices + per-slot sums)
  val s3_sum  = RegNext(s2_sum)
  val s3_idxs = RegNext(s2_idxs)
  val s3_meta = Wire(new GEHLMeta(nTables, nIdxBits, sumBits))
  s3_meta.sums := s3_sum
  s3_meta.idxs := s3_idxs
  io.f3_meta := s3_meta.asUInt

  // ============================================================
  // Update path: read-modify-write, pipelined over two cycles
  //   su1: decode train/direction from meta; issue SyncReadMem read of old weights
  //   su2: old weights land -> new = old +/-1 (saturating) -> masked write (slot w)
  // The SRAM read forces RMW: we can no longer read the old weight combinationally.
  // ============================================================
  val u_meta = s1_update.bits.meta.asTypeOf(new GEHLMeta(nTables, nIdxBits, sumBits))
  val u_fire = !doing_reset && s1_update.valid && s1_update.bits.is_commit_update

  // su1: per-slot train decision and resolved direction
  val su1_train = Wire(Vec(bankWidth, Bool()))
  val su1_taken = Wire(Vec(bankWidth, Bool()))
  for (w <- 0 until bankWidth) {
    val hit       = s1_update.bits.cfi_idx.valid && (s1_update.bits.cfi_idx.bits === w.U)
    val was_taken = hit && s1_update.bits.cfi_taken
    val mispred   = hit && s1_update.bits.cfi_mispredicted
    val stored    = u_meta.sums(w)
    val abs_sum   = Mux(stored >= 0.S, stored, -stored)
    su1_taken(w) := was_taken
    su1_train(w) := u_fire && s1_update.bits.br_mask(w) && (mispred || abs_sum < theta.S)
  }

  // su1: issue reads of the old weights at the stored indices -> data lands in su2
  val su2_old = VecInit((0 until nTables).map { t =>
    tables(t).read(u_meta.idxs(t), u_fire)
  })

  // stage control su1 -> su2 (aligns with su2_old)
  val su2_train = RegNext(su1_train)
  val su2_taken = RegNext(su1_taken)
  val su2_idxs  = RegNext(u_meta.idxs)

  // su2: compute saturating +/-1 new weights and per-slot write mask
  val su2_new   = Wire(Vec(nTables, Vec(bankWidth, SInt(weightBits.W))))
  val su2_wmask = Wire(Vec(nTables, Vec(bankWidth, Bool())))
  for (t <- 0 until nTables) {
    for (w <- 0 until bankWidth) {
      val old_w = su2_old(t)(w)
      su2_new(t)(w) := Mux(su2_taken(w),
        Mux(old_w === maxWeight, maxWeight, old_w + 1.S),
        Mux(old_w === minWeight, minWeight, old_w - 1.S))
      su2_wmask(t)(w) := su2_train(w)
    }
  }

  // ---- one write port per table: reset (priority) or update, masked per slot ----
  val zeroVec = VecInit(Seq.fill(bankWidth)(0.S(weightBits.W)))
  val fullMsk = VecInit(Seq.fill(bankWidth)(true.B))
  for (t <- 0 until nTables) {
    val wr_en   = doing_reset || su2_wmask(t).asUInt.orR
    val wr_idx  = Mux(doing_reset, reset_idx, su2_idxs(t))
    val wr_data = Mux(doing_reset, zeroVec,   su2_new(t))
    val wr_mask = Mux(doing_reset, fullMsk,   su2_wmask(t))
    when (wr_en) {
      tables(t).write(wr_idx, wr_data, (0 until bankWidth).map(i => wr_mask(i)))
    }
  }
}