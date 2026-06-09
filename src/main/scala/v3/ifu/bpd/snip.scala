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
  // Stage 4c: training + f3_meta carry + convergence observer.

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

  val coeffBias = 68
  val coeffs = VecInit(Seq(68, 56, 48, 37, 31, 24, 17, 17).map(_.S(8.W)))

  val weights = Seq.fill(Tbl)(SyncReadMem(E, Vec(F, SInt(5.W))))
  val biasMem = SyncReadMem(nbias, Vec(F, SInt(5.W)))

  val fp_untrained = ((1 << F) - 1).U(F.W)

  override val metaSz = 35  // valid(1) + min_ham(4) + sig(15) + fingerprint(15)

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
  def tblIdx(pc: UInt, ghist: UInt, t: Int): UInt =
    (fetchIdx(pc) ^ foldHist(ghist, histLens(t)))(log2Ceil(E) - 1, 0)

  // Target fingerprint extraction: 0xb5ffa skip bits → compacted 15-bit value
  def extractFp(target: UInt): UInt = {
    val bits = Seq(1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 16, 17, 19)
    VecInit(bits.map(b => target(b))).asUInt
  }

  // ── Weight/bias init FSM ────────────────────────────────────────────────────
  val wt_init_done = RegInit(false.B)
  val wt_init_idx  = RegInit(0.U(log2Ceil(nbias).W))
  val INIT_VAL = 0.S(5.W)
  val initRow = Wire(Vec(F, SInt(5.W))); initRow := VecInit(Seq.fill(F)(INIT_VAL))

  when (!wt_init_done) {
    wt_init_idx := wt_init_idx + 1.U
    when (wt_init_idx === (nbias - 1).U) { wt_init_done := true.B }
  }

  // ── Fingerprint dot-product (s1→s3) ─────────────────────────────────────────
  // PREDICT-SIDE sum_w formula (for bit-identity check with training):
  //   s2_sum(w) = Σ_t (s2_wt(t)(w) * coeffs(t)) + (s2_bia(w) * coeffBias)
  //   s3_sum = RegNext(s2_sum); fp_bit = (s3_sum(w) >= 0.S)
  val biasIdx = fetchIdx(s1_pc)(log2Ceil(nbias) - 1, 0)
  val readEn  = s1_valid && wt_init_done

  val s2_wt  = VecInit((0 until Tbl).map { t => weights(t).read(tblIdx(s1_pc, io.f1_ghist, t), readEn) })
  val s2_bia = biasMem.read(biasIdx, readEn)

  val s2_sum = VecInit((0 until F).map { w =>
    val wt = (0 until Tbl).map { t => (s2_wt(t)(w) * coeffs(t)).asSInt }.reduce(_ + _)
    (wt + (s2_bia(w) * coeffBias.S).asSInt).asSInt
  })

  val s3_sum = RegNext(s2_sum)
  val s3_fingerprint = VecInit(s3_sum.map(s => (s >= 0.S).asUInt)).asUInt

  // ── Commit-side Training RMW ────────────────────────────────────────────────
  val u        = io.update.bits
  val tr_fire  = io.update.valid && u.is_commit_update &&
                 u.cfi_is_jalr && !u.cfi_is_ret && u.cfi_taken && u.cfi_idx.valid

  // Issue reads at tr_fire (commit cycle, data 1 cycle later — same as ITC rd_rows)
  def tblIdxUpd(t: Int): UInt = tblIdx(u.pc, io.update.bits.ghist, t)
  val tr_rd      = VecInit((0 until Tbl).map(t => weights(t).read(tblIdxUpd(t), tr_fire)))
  val tr_bias_rd = biasMem.read(fetchIdx(u.pc)(log2Ceil(nbias) - 1, 0), tr_fire)

  // 1 cycle later: read data available (tr_rd/tr_bias_rd used DIRECTLY, no RegNext — ITC pattern)
  val tr_b_fire      = RegNext(tr_fire, false.B)
  val tr_b_target    = RegNext(u.target)
  val tr_b_fp        = RegNext(io.update.bits.meta(F - 1, 0))  // carried predict fingerprint
  val tr_b_idx       = RegInit(VecInit(Seq.fill(Tbl)(0.U(log2Ceil(E).W))))
  val tr_b_bias_idx  = RegInit(0.U(log2Ceil(nbias).W))
  when (tr_fire) {
    tr_b_idx      := VecInit((0 until Tbl).map(t => tblIdxUpd(t)))
    tr_b_bias_idx := fetchIdx(u.pc)(log2Ceil(nbias) - 1, 0)
  }

  // Training RMW at tr_b_fire: one write per table, all F bits updated in parallel
  val tr_actual_fp = extractFp(tr_b_target)

  // Per-bit train_we and updated rows (combinational, computed at tr_b_fire)
  // TRAINING-SIDE sum_w formula (must be bit-identical to predict-side s2_sum above):
  //   sum_w = Σ_t (tr_rd(t)(w) * coeffs(t)) + (tr_bias_rd(w) * coeffBias)
  val tr_we = Wire(Vec(F, Bool()))
  val tr_updated_wt = Wire(Vec(Tbl, Vec(F, SInt(5.W))))
  val tr_updated_bias = Wire(Vec(F, SInt(5.W)))

  for (w <- 0 until F) {
    val sum_w = (0 until Tbl).map { t =>
      (tr_rd(t)(w) * coeffs(t)).asSInt
    }.reduce(_ + _) + (tr_bias_rd(w) * coeffBias.S).asSInt
    val pred_bit   = (sum_w >= 0.S).asUInt
    val target_bit = tr_actual_fp(w)
    tr_we(w) := (pred_bit =/= target_bit)

    for (t <- 0 until Tbl) {
      val delta = Mux(target_bit.asBool, 1.S(5.W), -1.S(5.W))
      val raw   = Mux(tr_we(w), tr_rd(t)(w) + delta, tr_rd(t)(w))
      tr_updated_wt(t)(w) := Mux(raw > 15.S, 15.S, Mux(raw < -16.S, -16.S, raw))
    }
    val b_delta = Mux(target_bit.asBool, 1.S(5.W), -1.S(5.W))
    val b_raw   = Mux(tr_we(w), tr_bias_rd(w) + b_delta, tr_bias_rd(w))
    tr_updated_bias(w) := Mux(b_raw > 15.S, 15.S, Mux(b_raw < -16.S, -16.S, b_raw))
  }

  val tr_any_we = tr_we.asUInt.orR
  // val tr_any_we = false.B  // disable training for now, to save power and avoid interference with ITC testing

  // ── ONE muxed write port per mem (init || training), exactly ONE .write() call site each ──
  for (t <- 0 until Tbl) {
    val init_wr = !wt_init_done && (wt_init_idx < E.U)
    val wr_en   = init_wr || (tr_b_fire && tr_any_we)
    val wr_idx  = Mux(init_wr, wt_init_idx(log2Ceil(E)-1, 0), tr_b_idx(t))
    val wr_data = Mux(init_wr, initRow, tr_updated_wt(t))
    when (wr_en) { weights(t).write(wr_idx, wr_data) }
  }
  val bias_init = !wt_init_done
  val bwr_en    = bias_init || (tr_b_fire && tr_any_we)
  val bwr_idx   = Mux(bias_init, wt_init_idx, tr_b_bias_idx)
  val bwr_data  = Mux(bias_init, initRow, tr_updated_bias)
  when (bwr_en) { biasMem.write(bwr_idx, bwr_data) }

  // Convergence observer
  io.tr_event       := tr_b_fire
  io.tr_exact_event := tr_b_fire && (tr_b_fp === tr_actual_fp)

  // Hamming convergence observer (4c-obs)
  val tr_xor = Wire(UInt(F.W)); tr_xor := tr_b_fp ^ tr_actual_fp
  val tr_ham = PopCount(tr_xor)
  io.tr_ham_le2 := tr_b_fire && (tr_ham <= 2.U)
  io.tr_ham_le4 := tr_b_fire && (tr_ham <= 4.U)

  // 4d meta unpack + accuracy/confidence observer (commit-side)
  // Layout: meta(34)=valid, meta(33,30)=min_ham, meta(29,15)=sig, meta(14,0)=fingerprint(4c)
  val carried_snip_valid = RegNext(io.update.bits.meta(34))
  val carried_min_ham    = RegNext(io.update.bits.meta(33, 30))
  val carried_sig        = RegNext(io.update.bits.meta(29, 15))
  val actual_sig         = RegNext(u.target(F - 1, 0))

  io.snip_has_cand_event     := tr_b_fire && carried_snip_valid
  io.snip_pick_match_event   := tr_b_fire && carried_snip_valid && (carried_sig === actual_sig)
  io.snip_min_ham_le2_event  := tr_b_fire && carried_snip_valid && (carried_min_ham <= 2.U)
  io.snip_min_ham_le4_event  := tr_b_fire && carried_snip_valid && (carried_min_ham <= 4.U)

  // Shared set-index hash
  def itcSet(pc: UInt): UInt = fetchIdx(pc)(log2Ceil(itc_nSets) - 1, 0)

  // ── Commit-side ITC Update RMW ─────────────────────────────────────────────
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

  // ── Hamming-match + min-select (s3) ────────────────────────────────────────
  // JALR proxy: is_jal && taken. No per-slot is_jalr at predict (BTB stores only
  // is_br; is_jal = !is_br conflates JAL+JALR). Direct JALs have empty ITC columns
  // (never populated by the commit gate), so snip_valid stays false for them.
  val s3_jalr_mask = VecInit((0 until bankWidth).map(w =>
    s3_resp(w).is_jal && s3_resp(w).taken))
  val s3_has_jalr  = s3_jalr_mask.asUInt.orR
  val snip_col     = PriorityEncoder(s3_jalr_mask)

  val s3_pool = VecInit((0 until itc_nWays).map(w => s3_pred_rows(w)(snip_col)))
  val cand_fp    = VecInit(s3_pool.map(e => extractFp(e.target)))
  val cand_valid = VecInit(s3_pool.map(e => e.valid))

  val ham = VecInit((0 until itc_nWays).map { w =>
    PopCount((s3_fingerprint ^ cand_fp(w)).asUInt)  // 4 bits, 0..15
  })
  // Key: top bit = !valid (invalid sorts above all valid), lower bits = ham
  val key = VecInit((0 until itc_nWays).map { w =>
    Cat(!cand_valid(w), ham(w))  // 5-bit key
  })

  // Pairwise min-select (8-way) on key, tie-break lower way
  val k0 = Mux(key(0) <= key(1), 0.U, 1.U)
  val k1 = Mux(key(2) <= key(3), 2.U, 3.U)
  val k2 = Mux(key(4) <= key(5), 4.U, 5.U)
  val k3 = Mux(key(6) <= key(7), 6.U, 7.U)
  val k01 = Mux(key(k0) <= key(k1), k0, k1)
  val k23 = Mux(key(k2) <= key(k3), k2, k3)
  val snip_sel = Mux(key(k01) <= key(k23), k01, k23)

  val snip_valid   = cand_valid.asUInt.orR && s3_has_jalr
  val snip_target  = s3_pool(snip_sel).target
  val snip_sig     = snip_target(F - 1, 0)
  val snip_min_ham = ham(snip_sel)  // 4 bits, fits metaSz=35

  // Carry predict fingerprint + snip fields through f3_meta (TAGE pattern)
  // Layout [34:0]: valid(1) | min_ham(4) | sig(15) | fingerprint(15)
  // fingerprint at LSB [14:0] preserves 4c's tr_b_fp = meta(14,0)
  io.f3_meta := Cat(snip_valid, snip_min_ham, snip_sig, s3_fingerprint)

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
