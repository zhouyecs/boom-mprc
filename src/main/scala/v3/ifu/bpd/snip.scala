package boom.v3.ifu

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import boom.v3.common._

class SNIPBranchPredictorBank(implicit p: Parameters) extends BranchPredictorBank()(p) {
  // Stage 3a: inert pass-through.
  // metaSz = 0 inherited (no meta shift). io.resp := io.resp_in(0) inherited.
  // io.f3_meta := 0.U inherited. nInputs = 1 inherited.
  val mems = Nil
}
