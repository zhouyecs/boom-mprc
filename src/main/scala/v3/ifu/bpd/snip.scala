package boom.v3.ifu

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import boom.v3.common._

class SNIPBranchPredictorBank(implicit p: Parameters) extends BranchPredictorBank()(p) {
  // Stage 3a: inert pass-through — io.resp := io.resp_in(0) inherited.
  // Stage 3b: ITC population + candidate-pool observer.
  // Stage 4a: predict-side ITC read + 4 observer counters.

  val itc_nSets = 256
  val itc_nWays = 8

  class ITCEntry extends Bundle {
    val valid  = Bool()
    val target = UInt(vaddrBitsExtended.W)
    val nru    = Bool()
  }

  val itc = Seq.fill(itc_nWays) { SyncReadMem(itc_nSets, Vec(bankWidth, new ITCEntry)) }

  val mems = Seq.tabulate(itc_nWays)(w =>
    (s"snip_itc_way$w", itc_nSets, bankWidth * (1 + vaddrBitsExtended + 1)))

  // Shared set-index hash — single source of truth for commit and predict paths
  def itcSet(pc: UInt): UInt = fetchIdx(pc)(log2Ceil(itc_nSets) - 1, 0)

  // ── Commit-side Update RMW ────────────────────────────────────────────────
  val u        = io.update.bits
  val upd_fire = io.update.valid && u.is_commit_update &&
                 u.cfi_is_jalr && !u.cfi_is_ret && u.cfi_taken && u.cfi_idx.valid
  val upd_idx  = itcSet(u.pc)
  val upd_col  = u.cfi_idx.bits

  // Stage A: issue read (addr + enable together, SyncReadMem delays one cycle)
  val rd_rows = VecInit(itc.map(_.read(upd_idx, upd_fire)))

  // Stage B: read result available (memory already delayed by one cycle)
  val b_fire   = RegNext(upd_fire, false.B)
  val b_idx    = RegNext(upd_idx)
  val b_col    = RegNext(upd_col)
  val b_target = RegNext(u.target)
  val b_ways   = VecInit(rd_rows.map(r => r(b_col)))

  val b_hit_oh = VecInit(b_ways.map(w => w.valid && w.target === b_target))
  val b_hit    = b_hit_oh.asUInt.orR

  // Per-cycle 1-bit events for HPM (drive IO ports)
  io.itc_total_event := b_fire
  io.itc_hit_event   := b_fire && b_hit

  // NRU victim selection: prefer invalid → nru=0 → way 0; saturated sweep
  val inval_oh = VecInit(b_ways.map(w => !w.valid))
  val nru_oh   = VecInit(b_ways.map(w => !w.nru))
  val victim   = Mux(inval_oh.asUInt.orR, PriorityEncoder(inval_oh),
                 Mux(nru_oh.asUInt.orR,   PriorityEncoder(nru_oh), 0.U))
  val all_used = !inval_oh.asUInt.orR && !nru_oh.asUInt.orR

  // ── ITC Initialization FSM ─────────────────────────────────────────────────
  // SyncReadMem has no reset; Verilator random-inits it. Clear every entry once
  // at boot so the predict-side observer sees deterministic (empty) rows.
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

  // Per-way muxed write: init (boot clear) or commit RMW, one write port per way.
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
  // Issue full-row read at s0; SyncReadMem 1-cycle latency → data at s1
  val pred_rows = VecInit(itc.map(_.read(itcSet(io.f0_pc), io.f0_valid)))    // s1
  val s2_pred_rows = RegNext(pred_rows)      // s2
  val s3_pred_rows = RegNext(s2_pred_rows)   // s3 — aligns with resp_in(0).f3

  val s3_resp = io.resp_in(0).f3
  val s3_taken_mask = VecInit((0 until bankWidth).map(w => s3_resp(w).taken))
  val s3_has_taken = s3_taken_mask.asUInt.orR

  // Per-slot observer: each taken slot checks its OWN ITC column
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
