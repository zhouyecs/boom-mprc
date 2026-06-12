package boom.v3.ifu

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import boom.v3.common._

class BigITCPredictorBank(implicit p: Parameters) extends BranchPredictorBank()(p) {
  // PC-indexed set-associative indirect-target cache (tag-matched, no perceptron).
  // Baseline for equal-storage comparison: reassigns the perceptron BRAM budget
  // (weights+bias, 24 RAMB36) to a large tag-cache.

  def nSets   = p(BoomBigITCSets)   // default 2048
  def nWays   = p(BoomBigITCWays)   // default 8
  def tagBits = p(BoomBigITCTag)    // default 12

  class BigEntry extends Bundle {
    val valid  = Bool()
    val tag    = UInt(tagBits.W)
    val target = UInt(vaddrBitsExtended.W)
    val nru    = Bool()
  }

  // ── ITC memories: nWays parallel SyncReadMem, PC-indexed, full-entry write ──
  val itc = Seq.fill(nWays) { SyncReadMem(nSets, new BigEntry) }

  val W_ENTRY = 2 + vaddrBitsExtended + tagBits  // valid(1) + target + nru(1) + tag
  override val metaSz = nWays * W_ENTRY + log2Ceil(nSets)

  override val mems =
    Seq.tabulate(nWays)(w => (s"bigitc_way$w", nSets, W_ENTRY))

  // PC slicing (mirrors BTB index/tag pattern)
  def setIdx(pc: UInt): UInt = (pc >> log2Ceil(coreInstBytes))(log2Ceil(nSets) - 1, 0)
  def tagOf(pc: UInt):  UInt = (pc >> (log2Ceil(coreInstBytes) + log2Ceil(nSets)))(tagBits - 1, 0)

  // ── Predict-side: read at f0, data pipelines to s3 ─────────────────────────
  val s1_read_set = setIdx(io.f0_pc)
  val s1_ways = VecInit(itc.map(_.read(s1_read_set, io.f0_valid)))
  val s2_ways = RegNext(s1_ways)
  val s3_ways = RegNext(s2_ways)
  val s3_set  = RegNext(RegNext(s1_read_set))

  // Pipeline the f0 PC to s3 for tag computation
  val s3_pc = RegNext(RegNext(RegNext(io.f0_pc)))

  val s3_resp = io.resp_in(0).f3

  // Slot select: same proxy as SNIP (first is_jal && taken)
  val s3_jalr_mask = VecInit((0 until bankWidth).map(w =>
    s3_resp(w).is_jal && s3_resp(w).taken))
  val s3_has_jalr  = s3_jalr_mask.asUInt.orR
  val s3_slot      = PriorityEncoder(s3_jalr_mask)

  // Tag compare
  val s3_tag        = tagOf(s3_pc)
  val s3_hit_oh     = VecInit(s3_ways.map(w => w.valid && w.tag === s3_tag))
  val s3_hit        = s3_hit_oh.asUInt.orR
  val s3_hit_target = Mux1H(s3_hit_oh, s3_ways.map(_.target))

  // ── F3 target override (same hook as SNIP 4f) ──────────────────────────────
  when (s3_valid && s3_has_jalr && s3_hit) {
    io.resp.f3(s3_slot).predicted_pc.valid := true.B
    io.resp.f3(s3_slot).predicted_pc.bits  := s3_hit_target
  }

  // ── Meta carry: snapshot ways + set index (nWays*W_ENTRY + setBits = metaSz) ──
  io.f3_meta := Cat(s3_ways.asUInt, s3_set)

  // ── Commit-side update (meta-carry, no re-read → 1R1W) ──────────────────────
  val u        = io.update.bits
  val upd_fire = io.update.valid && u.is_commit_update &&
                 u.cfi_is_jalr && !u.cfi_is_ret && u.cfi_taken && u.cfi_idx.valid

  val cm_set = setIdx(u.pc)
  val cm_tag = tagOf(u.pc)

  val b_fire    = RegNext(upd_fire, false.B)
  val b_target  = RegNext(u.target)
  val b_set     = RegNext(cm_set)
  val b_tag     = RegNext(cm_tag)
  val b_c_set   = RegNext(io.update.bits.meta(log2Ceil(nSets) - 1, 0))
  val b_ways    = RegNext(io.update.bits.meta(metaSz - 1, log2Ceil(nSets)))
                    .asTypeOf(Vec(nWays, new BigEntry))

  val do_upd = b_fire && (b_set === b_c_set)

  // Reuse SNIP-style NRU/victim on the carried snapshot
  val b_hit_oh = VecInit(b_ways.map(w => w.valid && w.tag === b_tag))
  val b_hit    = b_hit_oh.asUInt.orR

  val inval_oh = VecInit(b_ways.map(w => !w.valid))
  val nru_oh   = VecInit(b_ways.map(w => !w.nru))
  val victim   = Mux(inval_oh.asUInt.orR, PriorityEncoder(inval_oh),
                 Mux(nru_oh.asUInt.orR,   PriorityEncoder(nru_oh), 0.U))
  val all_used = !inval_oh.asUInt.orR && !nru_oh.asUInt.orR

  // ── Init FSM ──────────────────────────────────────────────────────────────
  val init_done = RegInit(false.B)
  val init_idx  = RegInit(0.U(log2Ceil(nSets).W))
  val init_zero = {
    val e = Wire(new BigEntry); e.valid := false.B; e.tag := 0.U; e.target := 0.U; e.nru := false.B; e
  }
  val init_wr = !init_done

  when (!init_done) {
    init_idx := init_idx + 1.U
    when (init_idx === (nSets - 1).U) { init_done := true.B }
  }

  for (w <- 0 until nWays) {
    val is_victim = !b_hit && (w.U === victim)
    val is_hitway = b_hit && b_hit_oh(w)
    val clear_nru = all_used && !is_victim
    val commit_we = do_upd && (is_victim || is_hitway || clear_nru)

    val e = Wire(new BigEntry)
    e.valid  := true.B
    e.tag    := Mux(is_victim, b_tag, b_ways(w).tag)
    e.target := Mux(is_victim, b_target, b_ways(w).target)
    e.nru    := !clear_nru

    when (init_wr) {
      itc(w).write(init_idx, init_zero)
    }.elsewhen (commit_we) {
      itc(w).write(b_set, e)
    }
  }

  // ── Observers (same slots as SNIP for apples-to-apples comparison) ─────────
  // Predict-side
  io.itc_total_event := s3_valid && s3_has_jalr
  io.itc_hit_event   := s3_valid && s3_has_jalr && s3_hit
  io.pred_taken_event          := s3_valid && s3_has_jalr
  io.pred_pool_nonempty_event  := false.B  // no column pool
  io.pred_target_in_pool_event := false.B
  io.pred_pool_saturated_event := false.B
  io.fp_computed_event := false.B          // no perceptron
  io.fp_nonzero_event  := false.B

  // Commit-side
  io.tr_event       := b_fire
  io.tr_exact_event := b_fire && b_hit    // tag hit ≡ "exact" (training proxy)
  io.tr_ham_le2 := false.B
  io.tr_ham_le4 := false.B

  io.snip_has_cand_event     := b_fire && b_hit  // at least one valid entry
  io.snip_pick_match_event   := b_fire && b_hit &&
                                Mux1H(b_hit_oh, b_ways.map(_.target)) === b_target
  io.snip_min_ham_le2_event  := false.B
  io.snip_min_ham_le4_event  := false.B
}
