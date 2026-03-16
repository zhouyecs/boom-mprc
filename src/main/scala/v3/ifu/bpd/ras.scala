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

    val commit_valid = Input(Bool())
    val commit_idx   = Input(UInt(log2Ceil(nRasEntries).W))
    val commit_addr  = Input(UInt(vaddrBitsExtended.W))

    val repair_valid = Input(Bool())
  })
  val ras_pred = RegInit(VecInit(Seq.fill(nRasEntries)(0.U(vaddrBitsExtended.W))))
  val ras_wrb  = RegInit(VecInit(Seq.fill(nRasEntries)(0.U(vaddrBitsExtended.W))))

  io.read_addr := Mux(RegNext(io.write_valid && io.write_idx === io.read_idx),
    RegNext(io.write_addr),
    RegNext(ras_pred(io.read_idx)))

  when (io.write_valid) {
    ras_pred(io.write_idx) := io.write_addr
  }

  when (io.commit_valid) {
    ras_wrb(io.commit_idx) := io.commit_addr
  }

  when (io.repair_valid) {
    for (i <- 0 until nRasEntries) {
      ras_pred(i) := ras_wrb(i)
    }
  }

  if (IN_SIMULATION) {
    val ras_printf = PlusArg("ras-content", 0, "Print All RAS Content", 1)
    val (cycleCount, _) = Counter(true.B, Int.MaxValue)
    when (io.write_valid || io.commit_valid || io.repair_valid) {
      when (ras_printf(0)) {
        printf(p"[${cycleCount} RAS] Content Update:\n")
        for (i <- 0 until nRasEntries) {
          printf(p"  Entry $i: PRED=0x${Hexadecimal(ras_pred(i))} WRB=0x${Hexadecimal(ras_wrb(i))}\n")
          when (ras_pred(i) =/= ras_wrb(i)) {
            printf(p"  >>> Mismatch at entry $i: PRED=0x${Hexadecimal(ras_pred(i))} WRB=0x${Hexadecimal(ras_wrb(i))}\n")
          } .otherwise {
            printf(p"  >>> Match at entry $i: PRED=0x${Hexadecimal(ras_pred(i))} WRB=0x${Hexadecimal(ras_wrb(i))}\n")
          }
        }
      }
    }
  }

}

class RASBranchPredictorBank(params: BoomBTBParams = BoomBTBParams())(implicit p: Parameters) extends BranchPredictorBank()(p)
{
  require(nBanks == 1, "RAS predictor bank only supports single bank.")
  val mems = Seq(
    ("RAS_PRED", nRasEntries, vaddrBitsExtended),
    ("RAS_WRB",  nRasEntries, vaddrBitsExtended)
  )

  val ras = Module(new BoomRAS())

  ras.io.read_idx := io.f2_read_idx
  ras.io.write_valid := io.f3_write_valid
  ras.io.write_idx := io.f3_write_idx
  ras.io.write_addr := io.f3_write_addr

  // Repair logic
  // When a rob_flush is detected, we repair the PRED RAS from WRB RAS
  ras.io.repair_valid := io.update.valid && io.update.bits.is_rob_flush

  // Commit logic
  ras.io.commit_valid := io.update.valid && io.update.bits.is_commit_update && io.update.bits.cfi_is_call
  ras.io.commit_idx   := WrapInc(io.update.bits.ras_idx, nRasEntries)
  ras.io.commit_addr  := io.update.bits.pc + Mux(io.update.bits.cfi_is_rvc, 2.U, 4.U)

  for (i <- 0 until bankWidth) {
    io.resp.f3(i).ras_top := ras.io.read_addr
  }
}