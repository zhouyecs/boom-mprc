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

  def itc_nSets = p(BoomSnipITCSets)
  def itc_nWays = p(BoomSnipITCWays)
  def override_thresh = p(BoomSnipOverrideThresh)

  class ITCEntry extends Bundle {
    val valid  = Bool()
    val target = UInt(vaddrBitsExtended.W)
    val nru    = Bool()
  }

  // Folded Cat(set,col) index — full-entry write (no mask) → BRAM-compatible
  val itc = Seq.fill(itc_nWays) { SyncReadMem(itc_nSets * bankWidth, new ITCEntry) }
  def itcAddr(set: UInt, col: UInt): UInt = Cat(set, col(log2Ceil(bankWidth)-1, 0))

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

  // ── Meta layout constants (LSB-first) ─────────────────────────────────────
  // [W_FP-1:0] fp (15) | [W_SIG+W_FP-1:W_FP] sig (15) |
  // [W_HAM+W_SIG+W_FP-1:W_SIG+W_FP] min_ham (4) |
  // [W_BASE-1:W_HAM+W_SIG+W_FP] valid (1) → W_BASE = 35
  val W_FP = F; val W_SIG = F; val W_HAM = 4; val W_VALID = 1
  val W_BASE = W_FP + W_SIG + W_HAM + W_VALID  // 35
  val W_BIAS  = F * 5                              // 75
  val W_WT    = Tbl * F * 5                        // 600
  val W_ITC_ENTRY = 2 + vaddrBitsExtended   // valid(1) + target + nru(1)
  val W_POOL  = itc_nWays * W_ITC_ENTRY
  val offBias = W_BASE                             // 35
  val offWt   = offBias + W_BIAS                   // 110
  val offPool = offWt   + W_WT                     // 710
  // Attribution fields are appended above the existing layout so the low-bit
  // fingerprint/signature offsets remain stable.
  val offOverride   = offPool + W_POOL
  val offBaseValid  = offOverride + 1
  val offBaseTarget = offBaseValid + 1
  val offNewTarget  = offBaseTarget + vaddrBitsExtended

  override val metaSz = offNewTarget + vaddrBitsExtended
  require(metaSz <= bpdMaxMetaLength,
    s"SNIP metaSz ($metaSz) exceeds bpdMaxMetaLength ($bpdMaxMetaLength) " +
    s"— reduce itc_nWays ($itc_nWays) or increase bpdMaxMetaLength")

  override val mems =
    Seq.tabulate(itc_nWays)(w => (s"snip_itc_way$w", itc_nSets * bankWidth, 1 + vaddrBitsExtended + 1)) ++
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
  val s3_bia = RegNext(s2_bia)
  val s3_wt  = RegNext(s2_wt)
  val s3_fingerprint = VecInit(s3_sum.map(s => (s >= 0.S).asUInt)).asUInt

  // ── Commit-side Training RMW ────────────────────────────────────────────────
  val u        = io.update.bits
  val tr_fire  = io.update.valid && u.is_commit_update &&
                 u.cfi_is_jalr && !u.cfi_is_ret && u.cfi_taken && u.cfi_idx.valid

  // commit-side reads DROPPED — weights + ITC carried in meta (TAGE pattern)
  def tblIdxUpd(t: Int): UInt = tblIdx(u.pc, io.update.bits.ghist, t)

  // 1 cycle later: unpack carried snapshots
  val tr_b_fire      = RegNext(tr_fire, false.B)
  val tr_b_target    = RegNext(u.target)
  val tr_b_fp        = RegNext(io.update.bits.meta(F - 1, 0))
  val tr_carried_bia = RegNext(io.update.bits.meta(offBias + W_BIAS - 1, offBias)).asTypeOf(Vec(F, SInt(5.W)))
  val tr_carried_wt  = RegNext(io.update.bits.meta(offWt   + W_WT   - 1, offWt  )).asTypeOf(Vec(Tbl, Vec(F, SInt(5.W))))
  val tr_carried_pool= RegNext(io.update.bits.meta(offPool + W_POOL - 1, offPool)).asTypeOf(Vec(itc_nWays, new ITCEntry))
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
  //   sum_w = Σ_t (tr_carried_wt(t)(w) * coeffs(t)) + (tr_carried_bia(w) * coeffBias)
  val tr_we = Wire(Vec(F, Bool()))
  val tr_updated_wt = Wire(Vec(Tbl, Vec(F, SInt(5.W))))
  val tr_updated_bias = Wire(Vec(F, SInt(5.W)))

  for (w <- 0 until F) {
    val sum_w = (0 until Tbl).map { t =>
      (tr_carried_wt(t)(w) * coeffs(t)).asSInt
    }.reduce(_ + _) + (tr_carried_bia(w) * coeffBias.S).asSInt
    val pred_bit   = (sum_w >= 0.S).asUInt
    val target_bit = tr_actual_fp(w)
    tr_we(w) := (pred_bit =/= target_bit)

    for (t <- 0 until Tbl) {
      val delta = Mux(target_bit.asBool, 1.S(5.W), -1.S(5.W))
      val raw   = Mux(tr_we(w), tr_carried_wt(t)(w) + delta, tr_carried_wt(t)(w))
      tr_updated_wt(t)(w) := Mux(raw > 15.S, 15.S, Mux(raw < -16.S, -16.S, raw))
    }
    val b_delta = Mux(target_bit.asBool, 1.S(5.W), -1.S(5.W))
    val b_raw   = Mux(tr_we(w), tr_carried_bia(w) + b_delta, tr_carried_bia(w))
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

  // Compare the predecessor-chain target and the SNIP target against the full
  // architectural target. These four outcomes attribute only active overrides.
  val carried_override    = RegNext(io.update.bits.meta(offOverride))
  val carried_base_valid  = RegNext(io.update.bits.meta(offBaseValid))
  val carried_base_target = RegNext(io.update.bits.meta(offBaseTarget + vaddrBitsExtended - 1, offBaseTarget))
  val carried_new_target  = RegNext(io.update.bits.meta(offNewTarget + vaddrBitsExtended - 1, offNewTarget))
  val actual_target       = tr_b_target
  val attribution_fire    = tr_b_fire && carried_override
  val base_correct        = carried_base_valid && (carried_base_target === actual_target)
  val new_correct         = carried_new_target === actual_target
  val corrected           = attribution_fire && !base_correct && new_correct
  val harmed              = attribution_fire && base_correct && !new_correct
  val still_wrong         = attribution_fire && !base_correct && !new_correct
  val both_correct        = attribution_fire && base_correct && new_correct

  io.indirect_override_event    := attribution_fire
  io.indirect_corrected_event   := corrected
  io.indirect_harmed_event      := harmed
  io.indirect_still_wrong_event := still_wrong

  when (attribution_fire) {
    assert(PopCount(VecInit(Seq(corrected, harmed, still_wrong, both_correct))) === 1.U)
  }

  // Shared set-index hash
  def itcSet(pc: UInt): UInt = fetchIdx(pc)(log2Ceil(itc_nSets) - 1, 0)

  // ── Commit-side ITC Update RMW ─────────────────────────────────────────────
  val upd_fire = io.update.valid && u.is_commit_update &&
                 u.cfi_is_jalr && !u.cfi_is_ret && u.cfi_taken && u.cfi_idx.valid
  val upd_idx  = itcSet(u.pc)
  val upd_col  = u.cfi_idx.bits

  // ITC commit read DROPPED — carried in meta (TAGE pattern). Unpacked as tr_carried_pool.

  val b_fire   = RegNext(upd_fire, false.B)
  val b_idx    = RegNext(upd_idx)
  val b_col    = RegNext(upd_col)
  val b_target = RegNext(u.target)
  val b_ways   = tr_carried_pool

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
  val flatDepth = itc_nSets * bankWidth
  val init_done = RegInit(false.B)
  val init_idx  = RegInit(0.U(log2Ceil(flatDepth).W))
  val init_zero = {
    val e = Wire(new ITCEntry); e.valid := false.B; e.target := 0.U; e.nru := false.B; e
  }
  val init_wr = !init_done

  when (!init_done) {
    init_idx := init_idx + 1.U
    when (init_idx === (flatDepth - 1).U) { init_done := true.B }
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

    // Folded write: single call site, Muxed addr/data, no mask → 1R1W → BRAM
    val we    = init_wr || commit_we
    val waddr = Mux(init_wr, init_idx, itcAddr(b_idx, b_col))
    val wdata = Mux(init_wr, init_zero, e)
    when (we) { itc(w).write(waddr, wdata) }
  }

  // ── Predict-side ITC Read — folded: single-column pool directly ────────────
  val s3_resp = io.resp_in(0).f3
  // Derive snip_col at s2 from f2 response (is_jal stable s1→s3; taken stable
  // for non-BR entries). Read address = Cat(set, s2_snip_col) → single column.
  val s2_resp = io.resp_in(0).f2
  val s2_jalr_mask = VecInit((0 until bankWidth).map(w =>
    s2_resp(w).is_jal && s2_resp(w).taken))
  val s2_snip_col   = PriorityEncoder(s2_jalr_mask)
  val s2_itc_set    = RegNext(RegNext(itcSet(io.f0_pc)))  // double RegNext → s2 stage
  val s2_read_addr  = itcAddr(s2_itc_set, s2_snip_col)
  val s3_pool = VecInit((0 until itc_nWays).map(w =>
    itc(w).read(s2_read_addr, s2_valid)))

  // Carry s2_snip_col to s3 — one source, used for both read and override
  val snip_col     = RegNext(s2_snip_col)
  val s3_has_jalr  = RegNext(s2_jalr_mask.asUInt.orR)  // registered, aligned with snip_col

  val s3_taken_mask = VecInit((0 until bankWidth).map(w => s3_resp(w).taken))
  val s3_has_taken = s3_taken_mask.asUInt.orR

  // Fingerprint observer
  io.fp_computed_event := s3_valid && s3_has_taken
  io.fp_nonzero_event  := s3_valid && s3_has_taken && (s3_fingerprint =/= fp_untrained)

  // ── Hamming-match + min-select (s3) ────────────────────────────────────────
  // JALR proxy: is_jal && taken. Already computed at s2, carried via RegNext.
  val cand_fp    = VecInit(s3_pool.map(e => extractFp(e.target)))
  val cand_valid = VecInit(s3_pool.map(e => e.valid))

  val ham = VecInit((0 until itc_nWays).map { w =>
    PopCount((s3_fingerprint ^ cand_fp(w)).asUInt)  // 4 bits, 0..15
  })
  // Key: top bit = !valid (invalid sorts above all valid), lower bits = ham
  val key = VecInit((0 until itc_nWays).map { w =>
    Cat(!cand_valid(w), ham(w))  // 5-bit key
  })

  // Pairwise min-select over itc_nWays ways, tie-break lower way
  def reduceMin(pairs: Seq[(UInt, UInt)]): (UInt, UInt) = {
    if (pairs.length == 1) {
      pairs.head
    } else {
      val half = pairs.grouped(2).map { g =>
        val (a_idx, a_key) = g(0)
        val (b_idx, b_key) = if (g.length == 2) g(1) else (a_idx, a_key) // pad odd
        (Mux(b_key <= a_key, b_idx, a_idx), Mux(b_key <= a_key, b_key, a_key))
      }.toSeq
      reduceMin(half)
    }
  }
  val (snip_sel, _) = reduceMin((0 until itc_nWays).map(w =>
    (w.U(log2Ceil(itc_nWays).W), key(w))))

  val snip_valid   = cand_valid.asUInt.orR && s3_has_jalr
  val snip_target  = s3_pool(snip_sel).target
  val snip_sig     = snip_target(F - 1, 0)
  val snip_min_ham = ham(snip_sel)
  val base_prediction = s3_resp(snip_col).predicted_pc
  val snip_override = snip_valid && (snip_min_ham <= override_thresh.U)

  // Carry predict bias + weights + ITC pool + snip fields through f3_meta (TAGE pattern)
  // Layout: attribution | pool | wt | bias | valid|min_ham|sig|fingerprint
  io.f3_meta := Cat(snip_target, base_prediction.bits, base_prediction.valid, snip_override,
                    s3_pool.asUInt, s3_wt.asUInt, s3_bia.asUInt,
                    snip_valid, snip_min_ham, snip_sig, s3_fingerprint)

  // F3 target override: correct predicted_pc when confident (first active change)
  when (snip_override) {
    io.resp.f3(snip_col).predicted_pc.valid := true.B
    io.resp.f3(snip_col).predicted_pc.bits  := snip_target
  }

  // All-column observers dropped (folded read provides single column only)
  io.pred_taken_event          := s3_valid && s3_has_taken
  io.pred_pool_nonempty_event  := false.B
  io.pred_target_in_pool_event := false.B
  io.pred_pool_saturated_event := false.B
}
