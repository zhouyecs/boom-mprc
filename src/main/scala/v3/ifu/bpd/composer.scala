package boom.v3.ifu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.{Field, Parameters}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._

import boom.v3.common._
import boom.v3.util.{BoomCoreStringPrefix}


class ComposedBranchPredictorBank(implicit p: Parameters) extends BranchPredictorBank()(p)
{

  val (components, resp) = getBPDComponents(io.resp_in(0), p)
  io.resp := resp


  var metas = 0.U(1.W)
  var meta_sz = 0
  for (c <- components) {
    c.io.f0_valid  := io.f0_valid
    c.io.f0_pc     := io.f0_pc
    c.io.f0_mask   := io.f0_mask
    c.io.f1_ghist  := io.f1_ghist
    c.io.f1_lhist  := io.f1_lhist
    c.io.f3_fire   := io.f3_fire
    // For RAS
    c.io.f2_read_idx := io.f2_read_idx
    c.io.f3_write_valid := io.f3_write_valid
    c.io.f3_write_idx := io.f3_write_idx
    c.io.f3_write_addr := io.f3_write_addr
    if (c.metaSz > 0) {
      metas = (metas << c.metaSz) | c.io.f3_meta(c.metaSz-1,0)
    }
    meta_sz = meta_sz + c.metaSz
    println(s"BPD component: ${c.getClass.getSimpleName} meta size: ${c.metaSz}")
  }
  require(meta_sz < bpdMaxMetaLength)
  io.f3_meta := metas

  io.itc_total_event := components.map(_.io.itc_total_event).foldLeft(false.B)(_ || _)
  io.itc_hit_event   := components.map(_.io.itc_hit_event).foldLeft(false.B)(_ || _)
  io.pred_taken_event          := components.map(_.io.pred_taken_event).foldLeft(false.B)(_ || _)
  io.pred_pool_nonempty_event  := components.map(_.io.pred_pool_nonempty_event).foldLeft(false.B)(_ || _)
  io.pred_target_in_pool_event := components.map(_.io.pred_target_in_pool_event).foldLeft(false.B)(_ || _)
  io.pred_pool_saturated_event := components.map(_.io.pred_pool_saturated_event).foldLeft(false.B)(_ || _)

  var update_meta = io.update.bits.meta
  for (c <- components.reverse) {
    c.io.update := io.update
    c.io.update.bits.meta := update_meta
    update_meta = update_meta >> c.metaSz
  }

  val mems = components.map(_.mems).flatten

}
