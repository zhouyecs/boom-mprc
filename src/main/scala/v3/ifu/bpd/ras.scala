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
  })
  val ras = RegInit(VecInit(Seq.fill(nRasEntries)(0.U(vaddrBitsExtended.W))))

  io.read_addr := Mux(RegNext(io.write_valid && io.write_idx === io.read_idx),
    RegNext(io.write_addr),
    RegNext(ras(io.read_idx)))

  when (io.write_valid) {
    ras(io.write_idx) := io.write_addr
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

  for (i <- 0 until bankWidth) {
    io.resp.f3(i).ras_top := ras.io.read_addr
  }
}