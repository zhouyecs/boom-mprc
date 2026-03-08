//******************************************************************************
// Copyright (c) 2017 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

package boom.v3.ifu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config._
import freechips.rocketchip.subsystem._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.rocket._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.tile._
import freechips.rocketchip.util._
import freechips.rocketchip.util.property._


import boom.v3.common._
import boom.v3.exu.{CommitExceptionSignals, BranchDecode, BrUpdateInfo}
import boom.v3.util._

class BoomRAS(implicit p: Parameters) extends BoomModule()(p)
{
  val io = IO(new Bundle {
    val read_idx   = Input(UInt(log2Ceil(nRasEntries).W))
    val read_addr  = Output(UInt(vaddrBitsExtended.W))

    val write_valid = Input(Bool())
    val write_idx   = Input(UInt(log2Ceil(nRasEntries).W))
    val write_addr  = Input(UInt(vaddrBitsExtended.W))

    // SARAS logic signals
    val spec_pop_valid   = Input(Bool())
    val spec_pop_addr    = Input(UInt(vaddrBitsExtended.W))
    val spec_pop_idx     = Input(UInt(log2Ceil(nRasEntries).W))

    val commit_valid     = Input(Bool())
    val commit_is_call   = Input(Bool())
    val commit_is_ret    = Input(Bool())
    val commit_ras_idx   = Input(UInt(log2Ceil(nRasEntries).W))
    val commit_ras_addr  = Input(UInt(vaddrBitsExtended.W))

    val repair_valid     = Input(Bool())
    val tos_counter      = Output(UInt(log2Ceil(nRasEntries).W))
  })

  // Speculative RAS
  val ras = RegInit(VecInit(Seq.fill(nRasEntries)(0.U(vaddrBitsExtended.W))))

  io.read_addr := Mux(RegNext(io.write_valid && io.write_idx === io.read_idx),
    RegNext(io.write_addr),
    RegNext(ras(io.read_idx)))

  // Aligning Queue (AQ): 16 entries as per paper recommendation
  val aq_size = 16
  val aq_addr = Reg(Vec(aq_size, UInt(vaddrBitsExtended.W)))
  val aq_idx  = Reg(Vec(aq_size, UInt(log2Ceil(nRasEntries).W)))
  val aq_head = RegInit(0.U(log2Ceil(aq_size).W))
  val aq_tail = RegInit(0.U(log2Ceil(aq_size).W))

  // TOS Counter (the committed Top-of-Stack state)
  val tos_counter = RegInit(0.U(log2Ceil(nRasEntries).W))
  io.tos_counter := tos_counter

  // Speculative pushes
  when (io.write_valid) {
    ras(io.write_idx) := io.write_addr
  }

  // Speculative pops: record in AQ
  when (io.spec_pop_valid) {
    aq_addr(aq_tail) := io.spec_pop_addr
    aq_idx(aq_tail)  := io.spec_pop_idx
    aq_tail          := WrapInc(aq_tail, aq_size)
  }

  // Retirement: move AQ head and update TOS Counter
  when (io.commit_valid) {
    tos_counter := io.commit_ras_idx
    when (io.commit_is_ret) {
      aq_head     := WrapInc(aq_head, aq_size)
    }
  }

  // Repair content from AQ
  when (io.repair_valid) {
    // Write back AQ entries from latest (tail-1) down to earliest (head).
    // Chisel's last-assignment-wins ensures the EARLIEST pop (at aq_head) 
    // is the final value in the RAS for a given index.
    for (step <- 0 until aq_size) {
       val idx = (aq_tail - 1.U - step.U)
       val is_valid = Mux(aq_tail > aq_head,
         idx >= aq_head && idx < aq_tail,
         idx >= aq_head || idx < aq_tail)
       
       when (is_valid) {
         ras(aq_idx(idx)) := aq_addr(idx)
       }
    }
    aq_tail := aq_head
  }

  if (IN_SIMULATION) {
    val ras_printf = PlusArg("ras-content", 0, "Print All RAS Content", 1)
    val (cycleCount, _) = Counter(true.B, Int.MaxValue)
    when (ras_printf(0) && (io.write_valid || io.spec_pop_valid || io.repair_valid)) {
      printf(p"[${cycleCount} RAS] Updated. TOS Counter: ${tos_counter}\n")
    }
  }
}

class RASBranchPredictorBank(params: BoomBTBParams = BoomBTBParams())(implicit p: Parameters) extends BranchPredictorBank()(p)
{
  require(nBanks == 1, "RAS predictor bank only supports single bank.")
  val aq_size = 16
  val mems = Seq(
    ("RAS_TABLE", nRasEntries, vaddrBitsExtended),
    ("AQ_TABLE",  aq_size,     vaddrBitsExtended + log2Ceil(nRasEntries))
  )

  val ras = Module(new BoomRAS())

  ras.io.read_idx := io.f2_read_idx
  ras.io.write_valid := io.f3_write_valid
  ras.io.write_idx := io.f3_write_idx
  ras.io.write_addr := io.f3_write_addr

  ras.io.spec_pop_valid := io.f3_is_ret && io.f3_fire
  ras.io.spec_pop_addr  := io.f3_ras_top
  ras.io.spec_pop_idx   := io.f2_read_idx

  ras.io.commit_valid    := io.update.valid && io.update.bits.is_commit_update
  ras.io.commit_is_call  := io.update.bits.cfi_is_call
  ras.io.commit_is_ret   := io.update.bits.cfi_is_ret
  ras.io.commit_ras_idx  := io.update.bits.ras_idx
  ras.io.commit_ras_addr := io.update.bits.pc + Mux(io.update.bits.cfi_is_rvc, 2.U, 4.U)

  ras.io.repair_valid    := io.update.valid && (io.update.bits.is_mispredict_update || io.update.bits.is_rob_flush)

  io.tos_counter := ras.io.tos_counter

  for (i <- 0 until bankWidth) {
    io.resp.f3(i).ras_top := ras.io.read_addr
  }
}
