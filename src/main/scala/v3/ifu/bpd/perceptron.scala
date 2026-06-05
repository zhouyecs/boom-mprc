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
  // local history: repurpose tables 5,6,7 for local-history indexing
  localTables:      Seq[Int] = Seq(5, 6, 7),
  localHistLengths: Seq[Int] = Seq(8, 16, 32),
  lhtEntries:       Int = 256,
  lhtWidth:         Int = 32
) {
  require(nTables >= 2)
  require(isPow2(tableSize))
  require(localTables.length == localHistLengths.length)
  require(lhtWidth >= localHistLengths.maxOption.getOrElse(0),
    s"lhtWidth $lhtWidth too narrow for max localHistLength ${localHistLengths.maxOption.getOrElse(0)}")

  // geometric history lengths: table 0 = bias (len 0), tables 1..n-1 geometric minHist..maxHist
  def histLengths: Seq[Int] = {
    0 +: (1 until nTables).map { i =>
      val t = (i - 1).toDouble / (nTables - 2).toDouble
      round(minHist * pow(maxHist.toDouble / minHist.toDouble, t)).toInt
    }
  }

  def isLocalTable(t: Int): Boolean = localTables.contains(t)

  // Jimenez static threshold for training (TODO: make adaptive later)
  def theta: Int = floor(1.93 * maxHist).toInt + 14
}


// carried from predict to update: per-slot sums + shared per-table indices
// + override decision fields for confidence-gated GEHL-on-TAGE override
class GEHLMeta(val nTables: Int, val nIdxBits: Int, val sumBits: Int)(implicit p: Parameters) extends BoomBundle()(p)
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
  val lhtIdxBits = log2Ceil(params.lhtEntries)
  val lht = if (params.localTables.nonEmpty) Some(SyncReadMem(params.lhtEntries, UInt(params.lhtWidth.W))) else None
  val lhtWrBypassEntries = 2
  val mems = {
    val wm = (0 until nTables).map { t => (s"gehl_t$t", tableSize, bankWidth * weightBits) }
    if (params.localTables.nonEmpty) wm :+ ("gehl_lht", params.lhtEntries, params.lhtWidth) else wm
  }

  override val metaSz = bankWidth * sumBits + nTables * nIdxBits + 5 * bankWidth
  require(metaSz <= bpdMaxMetaLength)

  // s0 LHT read -> data lands at F1
  val s0_lht_idx = fetchIdx(io.f0_pc)(lhtIdxBits - 1, 0)
  val s1_lhist = Wire(UInt(params.lhtWidth.W))
  if (params.localTables.nonEmpty) {
    s1_lhist := lht.get.read(s0_lht_idx, s0_valid)
  } else {
    s1_lhist := 0.U
  }

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
    } else if (params.isLocalTable(t)) {
      val lhLen = params.localHistLengths(params.localTables.indexOf(t))
      val folded = foldHist(s1_lhist, lhLen, nIdxBits)
      s1_idxs(t) := (s1_pc_hash ^ folded)(nIdxBits - 1, 0)
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
    val would_fire_w = (abs_sum_w >= params.overrideTheta.S) &&
                       (gehl_pred_w =/= s3_incumbent(w)) && !s3_incumbent_conf(w)
    val fire_w       = would_fire_w && !params.observerMode.B
    assert(!would_fire_w || (gehl_pred_w =/= s3_incumbent(w)))
    io.resp.f3(w).taken := Mux(fire_w, gehl_pred_w, s3_incumbent(w))
  }

  // F3 meta (shared indices + per-slot sums + override decision)
  val s3_idxs = RegNext(s2_idxs)
  val s3_meta = Wire(new GEHLMeta(nTables, nIdxBits, sumBits))
  s3_meta.sums := s3_sum
  s3_meta.idxs := s3_idxs
  for (w <- 0 until bankWidth) {
    val gehl_pred_w  = s3_sum(w) >= 0.S
    val abs_sum_w    = Mux(s3_sum(w) >= 0.S, s3_sum(w), -s3_sum(w))
      s3_meta.would_fire(w) := (abs_sum_w >= params.overrideTheta.S) &&
                             (gehl_pred_w =/= s3_incumbent(w)) && !s3_incumbent_conf(w)
    s3_meta.gehl_pred(w)  := gehl_pred_w
    s3_meta.tage_pred(w)  := s3_incumbent(w)
    s3_meta.diag_incumbent_conf(w) := s3_incumbent_conf(w)
    s3_meta.diag_ungated_fire(w)   := (abs_sum_w >= params.overrideTheta.S) &&
                                      (gehl_pred_w =/= s3_incumbent(w))
  }
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

  // ---- Local History Table (LHT) commit write ----
  if (params.localTables.nonEmpty) {
    // su1: compute LHT index + per-slot outcome, issue read
    val su1_lht_idx   = fetchIdx(s1_update.bits.pc)(lhtIdxBits - 1, 0)
    val su1_any_br    = u_fire && s1_update.bits.br_mask.asUInt.orR
    val su1_cfi_valid = s1_update.bits.cfi_idx.valid
    val su1_cfi_idx   = s1_update.bits.cfi_idx.bits
    val su1_was_taken = Wire(Vec(bankWidth, Bool()))
    for (w <- 0 until bankWidth) {
      su1_was_taken(w) := su1_cfi_valid && (su1_cfi_idx === w.U) && s1_update.bits.cfi_taken
    }
    // SyncReadMem.read already lands at su2 -- do NOT RegNext.
    val su2_lht_old = this.lht.get.read(su1_lht_idx, su1_any_br)

    // pipeline su1 → su2
    val su2_any_br    = RegNext(su1_any_br)
    val su2_br_mask   = RegNext(s1_update.bits.br_mask)
    val su2_was_taken = RegNext(su1_was_taken)
    val su2_lht_idx   = RegNext(su1_lht_idx)
    val su2_cfi_valid = RegNext(su1_cfi_valid)
    val su2_cfi_idx   = RegNext(su1_cfi_idx)

    // LHT write-bypass (2 entries): covers read-during-write on back-to-back commits
    val lht_byp_idxs  = RegInit(VecInit(Seq.fill(lhtWrBypassEntries)(0.U(lhtIdxBits.W))))
    val lht_byp       = RegInit(VecInit(Seq.fill(lhtWrBypassEntries)(0.U(params.lhtWidth.W))))
    val lht_byp_enq   = RegInit(0.U(log2Ceil(lhtWrBypassEntries).W))
    val lht_byp_hits  = VecInit((0 until lhtWrBypassEntries).map { e =>
      !doing_reset && lht_byp_idxs(e) === su2_lht_idx
    })
    val lht_byp_hit   = lht_byp_hits.reduce(_||_)
    val lht_byp_hit_e = PriorityEncoder(lht_byp_hits)

    // su2: chain-shift each EXECUTED branch into the entry in program order (w=0 first).
    // Mux gates each shift in hardware; `executed` drops post-CFI phantom branches.
    var lht_val = Mux(lht_byp_hit, lht_byp(lht_byp_hit_e), su2_lht_old)
    for (w <- 0 until bankWidth) {
      val executed = !su2_cfi_valid || (w.U <= su2_cfi_idx)
      val do_shift = su2_any_br && su2_br_mask(w) && executed
      lht_val = Mux(do_shift,
        ((lht_val << 1) | su2_was_taken(w))(params.lhtWidth - 1, 0),
        lht_val)
    }
    val su2_lht_new = lht_val

    // write back + bypass update
    when (doing_reset) {
      this.lht.get.write(reset_idx(lhtIdxBits - 1, 0), 0.U(params.lhtWidth.W))
    }.elsewhen (su2_any_br) {
      this.lht.get.write(su2_lht_idx, su2_lht_new)
      when (lht_byp_hit) {
        lht_byp(lht_byp_hit_e) := su2_lht_new
      }.otherwise {
        lht_byp(lht_byp_enq)      := su2_lht_new
        lht_byp_idxs(lht_byp_enq) := su2_lht_idx
        lht_byp_enq := WrapInc(lht_byp_enq, lhtWrBypassEntries)
      }
    }
  }
}