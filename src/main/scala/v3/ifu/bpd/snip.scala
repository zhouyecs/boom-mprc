package boom.v3.ifu

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import boom.v3.common._

class SNIPBranchPredictorBank(implicit p: Parameters) extends BranchPredictorBank()(p) {
  // Stage 3a: inert pass-through — io.resp := io.resp_in(0) inherited.
  // Stage 3b: ITC population + candidate-pool observer.
  // Stage 4a: predict-side ITC read + 4 observer counters.
  // Stage 4b: fingerprint compute datapath (observer-only).

  val itc_nSets = 256
  val itc_nWays = 8

  class ITCEntry extends Bundle {
    val valid  = Bool()
    val target = UInt(vaddrBitsExtended.W)
    val nru    = Bool()
  }

  val itc = Seq.fill(itc_nWays) { SyncReadMem(itc_nSets, Vec(bankWidth, new ITCEntry)) }

  // ── Fingerprint datapath constants ──────────────────────────────────────────
  val F = 15
  val Tbl = 8
  val E = 1024
  val nbias = 4096

  val histLens = Seq(0, 2, 4, 8, 12, 18, 28, 42).map(_ min globalHistoryLength)

  // Frozen coeffs: sp[i]=max(1/(0.059+0.006*i),4.3) ×4. Need SInt(8.W) (68 > SInt(7) max 63).
  val coeffBias = 68
  val coeffs = VecInit(Seq(68, 56, 48, 37, 31, 24, 17, 17).map(_.S(8.W)))

  // T tables, F=15 bits packed per row — one read per table per predict (TAGE convention)
  val weights = Seq.fill(Tbl)(SyncReadMem(E, Vec(F, SInt(5.W))))
  val biasMem = SyncReadMem(nbias, Vec(F, SInt(5.W)))

  val fp_untrained = ((1 << F) - 1).U(F.W)  // 0x7FFF: all sums=0 → all bits 1

  override val mems =
    Seq.tabulate(itc_nWays)(w => (s"snip_itc_way$w", itc_nSets, bankWidth * (1 + vaddrBitsExtended + 1))) ++
    Seq.tabulate(Tbl)(t => (s"snip_weight$t", E, F * 5)) :+
    ("snip_bias", nbias, F * 5)

  // Index helpers
  def foldHist(hist: UInt, len: Int): UInt = {
    if (len == 0) 0.U(log2Ceil(E).W)
    else {
      val h = hist(len - 1, 0)
      val chunkW = log2Ceil(E)
      val nChunks = (len + chunkW - 1) / chunkW
      val chunks = (0 until nChunks).map { i =>
        h(Math.min((i + 1) * chunkW, len) - 1, i * chunkW)
      }
      chunks.reduce(_ ^ _)
    }
  }
  def tblIdx(pc: UInt, t: Int): UInt =
    (fetchIdx(pc) ^ foldHist(io.f1_ghist, histLens(t)))(log2Ceil(E) - 1, 0)

  // ── Weight/bias init FSM (TAGE/BTB/BIM doing_reset pattern) ──────────────
  val wt_init_done = RegInit(false.B)
  val wt_init_idx  = RegInit(0.U(log2Ceil(nbias).W))
  val INIT_VAL = 0.S(5.W)  // baseline; set to -1.S(5.W) for liveness probe

  when (!wt_init_done) {
    val initRow = Wire(Vec(F, SInt(5.W))); initRow := VecInit(Seq.fill(F)(INIT_VAL))
    for (t <- 0 until Tbl) {
      when (wt_init_idx < E.U) { weights(t).write(wt_init_idx, initRow) }
    }
    biasMem.write(wt_init_idx, initRow)
    wt_init_idx := wt_init_idx + 1.U
    when (wt_init_idx === (nbias - 1).U) { wt_init_done := true.B }
  }

  // ── Fingerprint dot-product (s1→s3) ───────────────────────────────────────
  // s1_pc and io.f1_ghist are both s1-aligned. Read at s1 → data s2 → 1 RegNext → s3.
  val biasIdx = fetchIdx(s1_pc)(log2Ceil(nbias) - 1, 0)
  val readEn  = s1_valid && wt_init_done

  val s2_wt  = VecInit((0 until Tbl).map { t => weights(t).read(tblIdx(s1_pc, t), readEn) })
  val s2_bia = biasMem.read(biasIdx, readEn)

  val s2_sum = VecInit((0 until F).map { w =>
    val wt = (0 until Tbl).map { t => (s2_wt(t)(w) * coeffs(t)).asSInt }.reduce(_ + _)
    (wt + (s2_bia(w) * coeffBias.S).asSInt).asSInt
  })

  val s3_sum = RegNext(s2_sum)
  val s3_fingerprint = VecInit(s3_sum.map(s => (s >= 0.S).asUInt)).asUInt

  // Shared set-index hash — single source of truth for commit and predict paths
  def itcSet(pc: UInt): UInt = fetchIdx(pc)(log2Ceil(itc_nSets) - 1, 0)

  // ── Commit-side Update RMW ────────────────────────────────────────────────
  val u        = io.update.bits
  val upd_fire = io.update.valid && u.is_commit_update &&
                 u.cfi_is_jalr && !u.cfi_is_ret && u.cfi_taken && u.cfi_idx.valid
  val upd_idx  = itcSet(u.pc)
  val upd_col  = u.cfi_idx.bits

  val rd_rows = VecInit(itc.map(_.read(upd_idx, upd_fire)))

  val b_fire   = RegNext(upd_fire, false.B)
  val b_idx    = RegNext(upd_idx)
  val b_col    = RegNext(upd_col)
  val b_target = RegNext(u.target)
  val b_ways   = VecInit(rd_rows.map(r => r(b_col)))

  val b_hit_oh = VecInit(b_ways.map(w => w.valid && w.target === b_target))
  val b_hit    = b_hit_oh.asUInt.orR

  io.itc_total_event := b_fire
  io.itc_hit_event   := b_fire && b_hit

  val inval_oh = VecInit(b_ways.map(w => !w.valid))
  val nru_oh   = VecInit(b_ways.map(w => !w.nru))
  val victim   = Mux(inval_oh.asUInt.orR, PriorityEncoder(inval_oh),
                 Mux(nru_oh.asUInt.orR,   PriorityEncoder(nru_oh), 0.U))
  val all_used = !inval_oh.asUInt.orR && !nru_oh.asUInt.orR

  // ── ITC Initialization FSM ─────────────────────────────────────────────────
  val init_done = RegInit(false.B)
  val init_idx  = RegInit(0.U(log2Ceil(itc_nSets).W))
  val init_zero = {
    val e = Wire(new ITCEntry); e.valid := false.B; e.target := 0.U; e.nru := false.B; e
  }
  val init_data = Wire(Vec(bankWidth, new ITCEntry))
  init_data := DontCare
  for (c <- 0 until bankWidth) { init_data(c) := init_zero }
  val init_mask = VecInit(Seq.fill(bankWidth)(true.B))

  when (!init_done) {
    init_idx := init_idx + 1.U
    when (init_idx === (itc_nSets - 1).U) { init_done := true.B }
  }

  for (w <- 0 until itc_nWays) {
    val is_victim = !b_hit && (w.U === victim)
    val is_hitway = b_hit && b_hit_oh(w)
    val clear_nru = all_used && !is_victim
    val commit_we = b_fire && (is_victim || is_hitway || clear_nru)

    val e = Wire(new ITCEntry)
    e.valid  := true.B
    e.target := Mux(is_victim, b_target, b_ways(w).target)
    e.nru    := !clear_nru
    val commit_row = Wire(Vec(bankWidth, new ITCEntry)); commit_row := DontCare; commit_row(b_col) := e
    val commit_mask = VecInit((0 until bankWidth).map(_.U === b_col))

    val we   = !init_done || commit_we
    val addr = Mux(init_done, b_idx,       init_idx)
    val data = Mux(init_done, commit_row,  init_data)
    val mask = Mux(init_done, commit_mask, init_mask)
    when (we) { itc(w).write(addr, data, mask) }
  }

  // ── Predict-side ITC Read (Observer) ──────────────────────────────────────
  val pred_rows = VecInit(itc.map(_.read(itcSet(io.f0_pc), io.f0_valid)))
  val s2_pred_rows = RegNext(pred_rows)
  val s3_pred_rows = RegNext(s2_pred_rows)

  val s3_resp = io.resp_in(0).f3
  val s3_taken_mask = VecInit((0 until bankWidth).map(w => s3_resp(w).taken))
  val s3_has_taken = s3_taken_mask.asUInt.orR

  // Fingerprint observer
  io.fp_computed_event := s3_valid && s3_has_taken
  io.fp_nonzero_event  := s3_valid && s3_has_taken && (s3_fingerprint =/= fp_untrained)

  val s3_slot_nonempty = VecInit((0 until bankWidth).map { w =>
    val col_w = s3_pred_rows.map(r => r(w))
    s3_resp(w).taken && col_w.map(_.valid).reduce(_ || _)
  })
  val s3_slot_hit = VecInit((0 until bankWidth).map { w =>
    val col_w = s3_pred_rows.map(r => r(w))
    val tgt_w = s3_resp(w).predicted_pc.bits
    s3_resp(w).taken && col_w.map(cw => cw.valid && cw.target === tgt_w).reduce(_ || _)
  })
  val s3_slot_saturated = VecInit((0 until bankWidth).map { w =>
    val col_w = s3_pred_rows.map(r => r(w))
    s3_resp(w).taken && col_w.map(_.valid).reduce(_ && _)
  })

  io.pred_taken_event          := s3_valid && s3_has_taken
  io.pred_pool_nonempty_event  := s3_valid && s3_slot_nonempty.asUInt.orR
  io.pred_target_in_pool_event := s3_valid && s3_slot_hit.asUInt.orR
  io.pred_pool_saturated_event := s3_valid && s3_slot_saturated.asUInt.orR
}
