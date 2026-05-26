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
    val read_corrupt = Output(Bool())

    val write_valid = Input(Bool())
    val write_idx   = Input(UInt(log2Ceil(nRasEntries).W))
    val write_addr  = Input(UInt(vaddrBitsExtended.W))

    // WP-RAS: Wrong-path corruption recovery
    val corrupt_set = Input(UInt(nRasEntries.W))  // Bitmask of entries to mark corrupted
  })
  val ras = RegInit(VecInit(Seq.fill(nRasEntries)(0.U(vaddrBitsExtended.W))))

  // WP-RAS: Corruption bits - one per RAS entry
  val corrupt = RegInit(0.U(nRasEntries.W))

  io.read_addr := Mux(RegNext(io.write_valid && io.write_idx === io.read_idx),
    RegNext(io.write_addr),
    RegNext(ras(io.read_idx)))

  // WP-RAS: Output whether the read entry is corrupted (with bypass like read_addr)
  io.read_corrupt := Mux(RegNext(io.write_valid && io.write_idx === io.read_idx),
    false.B,  // Just written by a call, definitely clean
    RegNext(corrupt(io.read_idx)))

  // WP-RAS: Update corruption bits
  val corrupt_clear = Mux(io.write_valid, 1.U(nRasEntries.W) << io.write_idx, 0.U(nRasEntries.W))
  corrupt := (corrupt | io.corrupt_set) & ~corrupt_clear

  when (io.write_valid) {
    ras(io.write_idx) := io.write_addr

    if (IN_SIMULATION) {
      val ras_printf = PlusArg("ras-content", 0, "Print All RAS Content", 1)
      val (cycleCount, _) = Counter(true.B, Int.MaxValue)
      when (ras_printf(0)) {
        printf(p"[${cycleCount} RAS] Content:\n")
        for (i <- 0 until nRasEntries) {
          printf(p"  Entry $i: addr=0x${Hexadecimal(ras(i))} corrupt=${corrupt(i)}\n")
        }
      }
    }
  }
}

class RASBranchPredictorBank(params: BoomBTBParams = BoomBTBParams())(implicit p: Parameters) extends BranchPredictorBank()(p)
{
  require(nBanks == 1, "RAS predictor bank only supports single bank.")
  val mems = Seq(("RASBranchPredictorBank", nRasEntries, vaddrBitsExtended))

  val ras = Module(new BoomRAS())

  ras.io.read_idx := io.f2_read_idx
  ras.io.write_valid := io.f3_write_valid
  ras.io.write_idx := io.f3_write_idx
  ras.io.write_addr := io.f3_write_addr
  ras.io.corrupt_set := io.ras_corrupt_set

  for (i <- 0 until bankWidth) {
    io.resp.f3(i).ras_top := ras.io.read_addr
    io.resp.f3(i).ras_corrupt := ras.io.read_corrupt
  }
}
