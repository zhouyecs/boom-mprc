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
  minHist:    Int = 4,
  overrideTheta: Int = 32,
  observerMode:  Boolean = true,
  f2Passthrough: Boolean = false, // true = corrector mode (pass incumbent F2)
  // training threshold: <0 => floor(1.93*nTables)+14 ; else fixed
  trainTheta:         Int   = -1,
  useAdaptiveTheta:   Boolean = false,
  tcBound:            Int     = 63,
  thetaMin:           Int     = 8,
  thetaMax:           Int     = 160,
  // override threshold: <0 => tracks training theta ; else fixed
  overrideThetaFixed: Int   = -1,
  // override-confidence table (SC-style learned gate)
  useOCT:             Boolean = true,
  octEntries:         Int     = 1024,
  octBits:            Int     = 4,
  octThreshold:       Int     = 0
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

  // training threshold: nTables-based (correct), overridable
  def trainThetaStatic: Int = if (trainTheta >= 0) trainTheta else floor(1.93 * nTables).toInt + 14
  // override theta: tracks trainTheta unless pinned
  def buildOverrideTheta(dyn: Int): Int = if (overrideThetaFixed >= 0) overrideThetaFixed else dyn
}


// carried from predict to update: per-slot sums + shared per-table indices
// + override decision fields for confidence-gated GEHL-on-TAGE override
class GEHLMeta(val nTables: Int, val nIdxBits: Int, val sumBits: Int, val octIdxBits: Int = 10)(implicit p: Parameters) extends BoomBundle()(p)
  with HasBoomFrontendParameters
{
  val sums       = Vec(bankWidth, SInt(sumBits.W))
  val idxs       = Vec(nTables, UInt(nIdxBits.W))
  val would_fire = Vec(bankWidth, Bool())
  val gehl_pred  = Vec(bankWidth, Bool())
  val tage_pred  = Vec(bankWidth, Bool())
  // TEMPORARY diagnostics
  val diag_incumbent_conf = Vec(bankWidth, Bool())
  val diag_ungated_fire   = Vec(bankWidth, Bool())
  // OCT liveness diagnostics
  val diag_oct_f3_nz  = Vec(bankWidth, Bool())
  val diag_oct_suppress = Vec(bankWidth, Bool())
  val diag_oct_write   = Vec(bankWidth, Bool())
  // meta-carried OCT index (block-level, not per-slot)
  val oct_idx = UInt(octIdxBits.W)
}


class GEHLBranchPredictorBank(params: BoomGEHLParams = BoomGEHLParams())(implicit p: Parameters) extends BranchPredictorBank()(p)
{
  val nTables    = params.nTables
  val tableSize  = params.tableSize
  val weightBits = params.weightBits
  val nIdxBits   = log2Ceil(tableSize)
  val histLens   = params.histLengths
  val trainThetaInit = params.trainThetaStatic

  // sum width: nTables * max_weight, plus sign
  val sumBits   = log2Ceil(nTables * ((1 << (weightBits - 1)) - 1) + 1) + 1
  val maxWeight = ((1 << (weightBits - 1)) - 1).S(weightBits.W)
  val minWeight = (-(1 << (weightBits - 1))).S(weightBits.W)

  // ---- adaptive training theta (O-GEHL) ----
  val theta_dyn = if (params.useAdaptiveTheta) RegInit(trainThetaInit.U(log2Ceil(params.thetaMax + 1).W)) else 0.U
  val tc = if (params.useAdaptiveTheta) RegInit(0.S(log2Ceil(params.tcBound + 1).W)) else 0.S
  val currentTheta = Mux(params.useAdaptiveTheta.B, theta_dyn, trainThetaInit.U)
  val overrideTheta = Mux((params.overrideThetaFixed >= 0).B,
    params.overrideThetaFixed.S, currentTheta.asSInt)

  // ---- OCT: flop-based learned gate (no SyncReadMem -> no CIRCT memory lowering) ----
  val octIdxBits = log2Ceil(params.octEntries)
  val octEntries = if (params.useOCT) params.octEntries else 1
  val oct = RegInit(VecInit(Seq.fill(octEntries)(0.S(params.octBits.W))))
  // F1 read -> pipeline to F3 (flop is 0-cycle; SRAM was 1-cycle, so add 2nd RegNext)
  // PC hash: XOR-fold fetchIdx to mix high bits into the index (avoids all-zero
  // low bits when fetchIdx sits at a large aligned base like 0x10000000)
  val oct_s1_idx = RegNext({
    val fc = fetchIdx(io.f0_pc)
    foldHist(fc, fc.getWidth, octIdxBits)
  })
  val oct_f3 = if (params.useOCT) {
    RegNext(RegNext(oct(oct_s1_idx)))
  } else { 0.S }

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

  override val metaSz = bankWidth * sumBits + nTables * nIdxBits + 8 * bankWidth + octIdxBits
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
    io.resp.f2(w).taken := Mux(params.f2Passthrough.B,
      io.resp_in(0).f2(w).taken,   // corrector: leave F2 to incumbent
      s2_resp(w))                  // standalone: GEHL drives F2
  }

  // side-channel confidence signals from TAGE and loop (F3, same cycle)
  val tage_provided = IO(Input(Vec(bankWidth, Bool())))
  val tage_strong   = IO(Input(Vec(bankWidth, Bool())))
  val loop_provided = IO(Input(Vec(bankWidth, Bool())))

  // pipeline s2→s3: GEHL sum + incumbent (loop/TAGE/BIM composite) prediction
  val s3_sum           = RegNext(s2_sum)
  val s3_incumbent     = VecInit((0 until bankWidth).map { w => io.resp_in(0).f3(w).taken })
  val s3_incumbent_conf = VecInit((0 until bankWidth).map { w =>
    loop_provided(w) || (tage_provided(w) && tage_strong(w))
  })

  for (w <- 0 until bankWidth) {
    val gehl_pred_w  = s3_sum(w) >= 0.S
    val abs_sum_w    = Mux(s3_sum(w) >= 0.S, s3_sum(w), -s3_sum(w))
    val oct_ok       = (!params.useOCT).B || (oct_f3 >= params.octThreshold.S)
    val would_fire_w = (abs_sum_w >= overrideTheta) &&
                       (gehl_pred_w =/= s3_incumbent(w)) && !s3_incumbent_conf(w) && oct_ok
    val fire_w       = would_fire_w && !params.observerMode.B
    assert(!would_fire_w || (gehl_pred_w =/= s3_incumbent(w)))
    io.resp.f3(w).taken := Mux(fire_w, gehl_pred_w, s3_incumbent(w))
  }

  // F3 meta (shared indices + per-slot sums + override decision)
  val s3_idxs = RegNext(s2_idxs)
  val s3_meta = Wire(new GEHLMeta(nTables, nIdxBits, sumBits, octIdxBits))
  s3_meta.sums := s3_sum
  s3_meta.idxs := s3_idxs
  for (w <- 0 until bankWidth) {
    val gehl_pred_w  = s3_sum(w) >= 0.S
    val abs_sum_w    = Mux(s3_sum(w) >= 0.S, s3_sum(w), -s3_sum(w))
    val oct_ok_w    = (!params.useOCT).B || (oct_f3 >= params.octThreshold.S)
    val pre_oct_w   = (abs_sum_w >= overrideTheta) &&
                      (gehl_pred_w =/= s3_incumbent(w)) && !s3_incumbent_conf(w)
    s3_meta.would_fire(w) := pre_oct_w && oct_ok_w
    s3_meta.gehl_pred(w)  := gehl_pred_w
    s3_meta.tage_pred(w)  := s3_incumbent(w)
    s3_meta.diag_incumbent_conf(w) := s3_incumbent_conf(w)
    s3_meta.diag_ungated_fire(w)   := (abs_sum_w >= overrideTheta) &&
                                      (gehl_pred_w =/= s3_incumbent(w))
    s3_meta.diag_oct_f3_nz(w)     := oct_f3 =/= 0.S
    s3_meta.diag_oct_suppress(w)  := pre_oct_w && !oct_ok_w
    s3_meta.diag_oct_write(w)     := pre_oct_w
  }
  // meta-carried OCT index: capture read index at F3 for commit-path use
  s3_meta.oct_idx := 0.U  // default (useOCT=false)
  if (params.useOCT) {
    s3_meta.oct_idx := RegNext(RegNext(oct_s1_idx))  // 3 deep from f0, same idx the read used
  }
  io.f3_meta := s3_meta.asUInt

  // ============================================================
  // Update path: read-modify-write, pipelined over two cycles
  //   su1: decode train/direction from meta; issue SyncReadMem read of old weights
  //   su2: old weights land -> new = old +/-1 (saturating) -> masked write (slot w)
  // The SRAM read forces RMW: we can no longer read the old weight combinationally.
  // ============================================================
  val u_meta = s1_update.bits.meta.asTypeOf(new GEHLMeta(nTables, nIdxBits, sumBits, octIdxBits))
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
    su1_train(w) := u_fire && s1_update.bits.br_mask(w) && (mispred || abs_sum < currentTheta.asSInt)
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

  // ---- adaptive training theta update (O-GEHL) ----
  if (params.useAdaptiveTheta) {
    val su1_trained  = Wire(Vec(bankWidth, Bool()))
    val su2_trained  = RegNext(su1_trained)
    val su2_mispred  = RegNext(s1_update.bits.cfi_mispredicted && s1_update.bits.cfi_idx.valid)
    val su2_any_tr   = su2_trained.asUInt.orR
    for (w <- 0 until bankWidth) {
      su1_trained(w) := su1_train(w)  // computed above at su1
    }
    val net_delta = PopCount(su2_trained.asUInt) // +1 per trained slot
    val tc_next = tc + Mux(su2_mispred, net_delta.asSInt, -net_delta.asSInt)
    when (su2_any_tr) {
      tc := tc_next
      when (tc_next >= params.tcBound.S) {
        theta_dyn := Mux(theta_dyn < params.thetaMax.U, theta_dyn + 1.U, theta_dyn)
        tc := 0.S
      }.elsewhen (tc_next <= (-params.tcBound).S) {
        theta_dyn := Mux(theta_dyn > params.thetaMin.U, theta_dyn - 1.U, theta_dyn)
        tc := 0.S
      }
    }
  }

  // ---- OCT commit write (SC-style learned gate, flop-based) ----
  if (params.useOCT) {
    // use F3-carried pre-oct decision, gated to real committed branches
    val su1_pre_fire = Wire(Vec(bankWidth, Bool()))
    for (w <- 0 until bankWidth) {
      su1_pre_fire(w) := u_fire && s1_update.bits.br_mask(w) && u_meta.diag_oct_write(w)
    }
    val su2_pre_fire  = RegNext(su1_pre_fire)
    val su2_gehl_pred = RegNext(u_meta.gehl_pred)
    val su2_oct_idx   = RegNext(u_meta.oct_idx)
    val su2_actual    = RegNext(su1_taken)

    // combinational reg read at su2 (always latest committed value)
    val su2_oct_old = oct(su2_oct_idx)

    val maxOct = ((1 << (params.octBits - 1)) - 1).S
    val minOct = (-(1 << (params.octBits - 1))).S
    var oct_val: SInt = su2_oct_old
    for (w <- 0 until bankWidth) {
      val fix = su2_gehl_pred(w) === su2_actual(w)
      val inc = Mux(fix && oct_val === maxOct, 0.S,
                 Mux(!fix && oct_val === minOct, 0.S,
                 Mux(fix, 1.S, -1.S)))
      oct_val = Mux(su2_pre_fire(w), oct_val + inc, oct_val)
    }
    when (su2_pre_fire.asUInt.orR) {
      oct(su2_oct_idx) := oct_val
    }
  }
}