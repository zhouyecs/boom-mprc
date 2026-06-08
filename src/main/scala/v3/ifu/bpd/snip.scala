package boom.v3.ifu

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import boom.v3.common._

class SNIPBranchPredictorBank(implicit p: Parameters) extends BranchPredictorBank()(p) {
  // Stage 3a: inert pass-through — io.resp := io.resp_in(0) inherited.
  // Stage 3b: ITC population + candidate-pool observer.

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

  // ── Update RMW (BTB-mirrored: per-way masked write, 1-cycle delay) ──────
  val u        = io.update.bits
  val upd_fire = io.update.valid && u.is_commit_update &&
                 u.cfi_is_jalr && !u.cfi_is_ret && u.cfi_taken && u.cfi_idx.valid
  val upd_idx  = fetchIdx(u.pc)(log2Ceil(itc_nSets) - 1, 0)
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

  // Masked per-way write-back (BTB idiom)
  for (w <- 0 until itc_nWays) {
    val is_victim = !b_hit && (w.U === victim)
    val is_hitway = b_hit && b_hit_oh(w)
    val clear_nru = all_used && !is_victim
    when (b_fire && (is_victim || is_hitway || clear_nru)) {
      val e = Wire(new ITCEntry)
      e.valid  := true.B
      e.target := Mux(is_victim, b_target, b_ways(w).target)
      e.nru    := !clear_nru
      val data = Wire(Vec(bankWidth, new ITCEntry)); data := DontCare; data(b_col) := e
      itc(w).write(b_idx, data, VecInit((0 until bankWidth).map(_.U === b_col)))
    }
  }
}
