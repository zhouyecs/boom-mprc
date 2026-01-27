// Return Address Stack (RAS) for BOOM V3.
// Uses a commit stack + speculative queue architecture with
// counter-based call compression and linked-list next-on-stack
// pointers for O(1) recovery on mispredictions.


package boom.v3.ifu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.{Parameters}
import freechips.rocketchip.subsystem.{TilesLocated, InSubsystem}

import boom.v3.common._
import boom.v3.util._

// ---------------------------------------------------------------------------
// Bundles
// ---------------------------------------------------------------------------

class RasEntry(implicit p: Parameters) extends BoomBundle
  with HasBoomCoreParameters
{
  val returnAddr = UInt(vaddrBitsExtended.W)
  val counter    = UInt(rasCounterWidth.W)
}

object RasEntry {
  def apply(returnAddr: UInt, counter: UInt)(implicit p: Parameters): RasEntry = {
    val e = Wire(new RasEntry)
    e.returnAddr := returnAddr
    e.counter    := counter
    e
  }
}

class RasQueuePtr(entries: Int)(implicit p: Parameters) extends CircularQueuePtr[RasQueuePtr](entries)
{
  def this()(implicit p: Parameters) = this(
    p(TilesLocated(freechips.rocketchip.subsystem.InSubsystem)).head
      .tileParams.core.asInstanceOf[BoomCoreParams].rasSpecQueueSize
  )
}

object RasQueuePtr {
  def apply(f: Bool, v: UInt)(implicit p: Parameters): RasQueuePtr = {
    val ptr = Wire(new RasQueuePtr)
    ptr.flag  := f
    ptr.value := v
    ptr
  }
}

class RasMeta(implicit p: Parameters) extends BoomBundle
  with HasBoomCoreParameters
{
  val specStackPtr = UInt(log2Up(rasCommitStackSize).W)
  val specCounter  = UInt(rasCounterWidth.W)
  val specReadPtr  = new RasQueuePtr
  val specWritePtr = new RasQueuePtr
  val nextOnStack  = new RasQueuePtr
}

class RasRedirect(implicit p: Parameters) extends BoomBundle
  with HasBoomCoreParameters
{
  val isCall   = Bool()
  val isRet    = Bool()
  val callAddr = UInt(vaddrBitsExtended.W)
  val meta     = new RasMeta
}

class RasCommit(implicit p: Parameters) extends BoomBundle
  with HasBoomCoreParameters
{
  val isCall       = Bool()
  val isRet        = Bool()
  val metaWritePtr = new RasQueuePtr
  val metaStackPtr = UInt(log2Up(rasCommitStackSize).W)
}

// ---------------------------------------------------------------------------
// BoomRasStack — the main dual-structure RAS module
// ---------------------------------------------------------------------------

class BoomRasStack(implicit p: Parameters) extends BoomModule
  with HasBoomCoreParameters
  with HasBoomFrontendParameters
  with HasCircularQueuePtrHelper
{
  val io = IO(new Bundle {
    val spec = new Bundle {
      val fire      = Input(Bool())
      val pushValid = Input(Bool())
      val popValid  = Input(Bool())
      val pushAddr  = Input(UInt(vaddrBitsExtended.W))
      val popAddr   = Output(UInt(vaddrBitsExtended.W))
    }
    val redirect = new Bundle {
      val valid    = Input(Bool())
      val isCall   = Input(Bool())
      val isRet    = Input(Bool())
      val callAddr = Input(UInt(vaddrBitsExtended.W))
      val meta     = Input(new RasMeta)
    }
    val commit = Flipped(Valid(new RasCommit))
    val meta          = Output(new RasMeta)
    val specNearOverflow = Output(Bool())
  })

  val counterMax = (1 << rasCounterWidth) - 1

  // Storage
  private val committedReturnAddressStack = RegInit(VecInit(Seq.fill(rasCommitStackSize)(
    RasEntry(0.U(vaddrBitsExtended.W), 0.U))))
  private val speculativeReturnAddressQueue = RegInit(VecInit(Seq.fill(rasSpecQueueSize)(
    RasEntry(0.U(vaddrBitsExtended.W), 0.U))))
  private val specNextOnStack = RegInit(VecInit(Seq.fill(rasSpecQueueSize)(
    RasQueuePtr(false.B, 0.U))))

  // Pointers
  private val committedReturnAddressStackPtr = RegInit(0.U(log2Up(rasCommitStackSize).W))
  private val specStackPtr   = RegInit(0.U(log2Up(rasCommitStackSize).W))
  private val specCounter    = RegInit(0.U(rasCounterWidth.W))
  private val specReadPtr    = RegInit(RasQueuePtr(true.B, (rasSpecQueueSize - 1).U))
  private val specWritePtr   = RegInit(RasQueuePtr(false.B, 0.U))
  private val specBasePtr    = RegInit(RasQueuePtr(false.B, 0.U))

  private val nearOverflow = RegInit(false.B)

  // Write bypass
  private val bypassEntry       = Reg(new RasEntry)
  private val bypassNextPtr     = Reg(new RasQueuePtr)
  private val bypassValid       = RegInit(false.B)
  private val bypassValidWire   = Wire(Bool())

  // Helper: pointer arithmetic for commit stack (simple wrapping)
  private def incCommitStackPtr(ptr: UInt): UInt = {
    if (isPow2(rasCommitStackSize)) (ptr + 1.U)(log2Up(rasCommitStackSize)-1, 0)
    else Mux(ptr === (rasCommitStackSize - 1).U, 0.U, ptr + 1.U)
  }
  private def decCommitStackPtr(ptr: UInt): UInt = {
    if (isPow2(rasCommitStackSize)) (ptr - 1.U)(log2Up(rasCommitStackSize)-1, 0)
    else Mux(ptr === 0.U, (rasCommitStackSize - 1).U, ptr - 1.U)
  }
  private def incQueuePtr(ptr: RasQueuePtr): RasQueuePtr = ptr + 1.U
  private def decQueuePtr(ptr: RasQueuePtr): RasQueuePtr = ptr - 1.U

  // Check if specReadPtr is within the valid range of the speculative queue
  def specReadPtrInRange(currReadPtr: RasQueuePtr, currWritePtr: RasQueuePtr): Bool = {
    !isBefore(currReadPtr, specBasePtr) && isBefore(currReadPtr, currWritePtr)
  }

  def committedReturnAddressStackTop(currStackPtr: UInt): RasEntry = committedReturnAddressStack(currStackPtr)

  def readNextOnStack(currReadPtr: RasQueuePtr, allowBypass: Boolean): RasQueuePtr = {
    val ret = Wire(new RasQueuePtr)
    if (allowBypass) {
      when(bypassValid) {
        ret := bypassNextPtr
      }.otherwise {
        ret := specNextOnStack(currReadPtr.value)
      }
    } else {
      ret := specNextOnStack(currReadPtr.value)
    }
    ret
  }

  def readStackTop(currStackPtr: UInt, currCounter: UInt, currReadPtr: RasQueuePtr,
                   currWritePtr: RasQueuePtr, allowBypass: Boolean): RasEntry = {
    val ret = Wire(new RasEntry)
    if (allowBypass) {
      when(bypassValid) {
        ret := bypassEntry
      }.elsewhen(specReadPtrInRange(currReadPtr, currWritePtr)) {
        ret := speculativeReturnAddressQueue(currReadPtr.value)
      }.otherwise {
        ret := committedReturnAddressStackTop(currStackPtr)
      }
    } else {
      when(specReadPtrInRange(currReadPtr, currWritePtr)) {
        ret := speculativeReturnAddressQueue(currReadPtr.value)
      }.otherwise {
        ret := committedReturnAddressStackTop(currStackPtr)
      }
    }
    ret
  }

  // Speculative push: update pointers, write to speculative queue happens next cycle
  def speculativePush(
    returnAddr:    UInt,
    currStackPtr:  UInt,
    currCounter:   UInt,
    currReadPtr:   RasQueuePtr,
    currWritePtr:  RasQueuePtr,
    topEntry:      RasEntry
  ): Unit = {
    specReadPtr  := currWritePtr
    specWritePtr := incQueuePtr(currWritePtr)
    when(topEntry.returnAddr === returnAddr && currCounter < counterMax.U) {
      specCounter := currCounter + 1.U
    }.otherwise {
      specStackPtr := incCommitStackPtr(currStackPtr)
      specCounter  := 0.U
    }
  }

  // Speculative pop: update pointers
  def speculativePop(currStackPtr: UInt, currCounter: UInt, currReadPtr: RasQueuePtr,
                     currWritePtr: RasQueuePtr, currTopNextPtr: RasQueuePtr): Unit = {
    when(specReadPtrInRange(currReadPtr, currWritePtr)) {
      specReadPtr := currTopNextPtr
    }
    when(currCounter > 0.U) {
      specCounter := currCounter - 1.U
    }.elsewhen(specReadPtrInRange(currTopNextPtr, currWritePtr)) {
      specStackPtr := decCommitStackPtr(currStackPtr)
      specCounter  := speculativeReturnAddressQueue(currTopNextPtr.value).counter
    }.otherwise {
      specStackPtr := decCommitStackPtr(currStackPtr)
      specCounter  := committedReturnAddressStackTop(decCommitStackPtr(currStackPtr)).counter
    }
  }

  // -----------------------------------------------------------------------
  // Write bypass valid logic
  // -----------------------------------------------------------------------
  when(io.redirect.valid && io.redirect.isCall) {
    bypassValidWire := true.B
    bypassValid     := true.B
  }.elsewhen(io.redirect.valid) {
    bypassValidWire := false.B
    bypassValid     := false.B
  }.elsewhen(io.spec.fire) {
    bypassValidWire := io.spec.pushValid
    bypassValid     := io.spec.pushValid
  }.otherwise {
    bypassValidWire := bypassValid
    bypassValid     := false.B
  }

  // -----------------------------------------------------------------------
  // Current top entry and next-on-stack (with bypass)
  // -----------------------------------------------------------------------
  private val topEntry = readStackTop(specStackPtr, specCounter,
    specReadPtr, specWritePtr, allowBypass = true)
  private val topNextPtr = readNextOnStack(specReadPtr, allowBypass = true)
  private val redirectTopEntry = readStackTop(
    io.redirect.meta.specStackPtr, io.redirect.meta.specCounter,
    io.redirect.meta.specReadPtr, io.redirect.meta.specWritePtr,
    allowBypass = false)
  private val redirectNextPtr = io.redirect.meta.nextOnStack

  // -----------------------------------------------------------------------
  // Write entry computation (for speculative push or redirect call)
  // -----------------------------------------------------------------------
  private val writeEntry    = Wire(new RasEntry)
  private val writeNextPtr  = Wire(new RasQueuePtr)
  writeEntry.returnAddr := Mux(io.redirect.valid && io.redirect.isCall,
    io.redirect.callAddr, io.spec.pushAddr)
  writeEntry.counter := Mux(io.redirect.valid && io.redirect.isCall,
    Mux(redirectTopEntry.returnAddr === io.redirect.callAddr &&
        redirectTopEntry.counter < counterMax.U,
      io.redirect.meta.specCounter + 1.U, 0.U),
    Mux(topEntry.returnAddr === io.spec.pushAddr &&
        topEntry.counter < counterMax.U,
      specCounter + 1.U, 0.U))
  writeNextPtr := Mux(io.redirect.valid && io.redirect.isCall,
    io.redirect.meta.specReadPtr, specReadPtr)

  when(io.spec.pushValid || (io.redirect.valid && io.redirect.isCall)) {
    bypassEntry   := writeEntry
    bypassNextPtr := writeNextPtr
  }

  // -----------------------------------------------------------------------
  // predictedTop: pre-computed next-cycle top for timing closure
  // -----------------------------------------------------------------------
  private val delayedPush       = Wire(Bool())
  private val delayedWriteEntry = Wire(new RasEntry)
  private val predictedTop      = RegInit(0.U.asTypeOf(new RasEntry))

  when(bypassValidWire) {
    when((io.redirect.valid && io.redirect.isCall) || io.spec.pushValid) {
      predictedTop := writeEntry
    }.otherwise {
      predictedTop := bypassEntry
    }
  }.elsewhen(io.redirect.valid && io.redirect.isRet) {
    val popRedStackPtr = Wire(UInt(log2Up(rasCommitStackSize).W))
    val popRedCounter  = Wire(UInt(rasCounterWidth.W))
    val popRedReadPtr  = io.redirect.meta.nextOnStack
    val popRedWritePtr = io.redirect.meta.specWritePtr
    when(io.redirect.meta.specCounter > 0.U) {
      popRedCounter  := io.redirect.meta.specCounter - 1.U
      popRedStackPtr := io.redirect.meta.specStackPtr
    }.elsewhen(specReadPtrInRange(popRedReadPtr, specWritePtr)) {
      popRedStackPtr := decCommitStackPtr(io.redirect.meta.specStackPtr)
      popRedCounter  := speculativeReturnAddressQueue(popRedReadPtr.value).counter
    }.otherwise {
      popRedStackPtr := decCommitStackPtr(io.redirect.meta.specStackPtr)
      popRedCounter  := committedReturnAddressStackTop(decCommitStackPtr(io.redirect.meta.specStackPtr)).counter
    }
    predictedTop := readStackTop(popRedStackPtr, popRedCounter,
      popRedReadPtr, popRedWritePtr, allowBypass = false)
  }.elsewhen(io.redirect.valid) {
    // Neither call nor ret — just restore
    predictedTop := readStackTop(io.redirect.meta.specStackPtr,
      io.redirect.meta.specCounter,
      io.redirect.meta.specReadPtr, io.redirect.meta.specWritePtr,
      allowBypass = false)
  }.elsewhen(io.spec.popValid) {
    val popStackPtr = Wire(UInt(log2Up(rasCommitStackSize).W))
    val popCounter  = Wire(UInt(rasCounterWidth.W))
    val popReadPtr  = topNextPtr
    val popWritePtr = specWritePtr
    when(specCounter > 0.U) {
      popCounter  := specCounter - 1.U
      popStackPtr := specStackPtr
    }.elsewhen(specReadPtrInRange(popReadPtr, specWritePtr)) {
      popStackPtr := decCommitStackPtr(specStackPtr)
      popCounter  := speculativeReturnAddressQueue(popReadPtr.value).counter
    }.otherwise {
      popStackPtr := decCommitStackPtr(specStackPtr)
      popCounter  := committedReturnAddressStackTop(decCommitStackPtr(specStackPtr)).counter
    }
    predictedTop := readStackTop(popStackPtr, popCounter,
      popReadPtr, popWritePtr, allowBypass = false)
  }.elsewhen(delayedPush) {
    predictedTop := delayedWriteEntry
  }.otherwise {
    predictedTop := readStackTop(specStackPtr, specCounter,
      specReadPtr, specWritePtr, allowBypass = false)
  }

  // -----------------------------------------------------------------------
  // Delayed write to speculative queue (one cycle delay for timing)
  // -----------------------------------------------------------------------
  delayedWriteEntry := RegEnable(writeEntry,
    io.spec.fire || io.redirect.isCall)

  private val delayedWriteAddr = RegEnable(
    Mux(io.redirect.valid && io.redirect.isCall,
      io.redirect.meta.specWritePtr, specWritePtr),
    io.spec.fire || (io.redirect.valid && io.redirect.isCall))

  private val delayedNextPtr = RegEnable(
    Mux(io.redirect.valid && io.redirect.isCall,
      io.redirect.meta.specReadPtr, specReadPtr),
    io.spec.fire || (io.redirect.valid && io.redirect.isCall))

  delayedPush := RegNext(io.spec.pushValid, init = false.B) ||
    RegNext(io.redirect.valid && io.redirect.isCall, init = false.B)

  when(delayedPush) {
    speculativeReturnAddressQueue(delayedWriteAddr.value) := delayedWriteEntry
    specNextOnStack(delayedWriteAddr.value)  := delayedNextPtr
  }

  // -----------------------------------------------------------------------
  // Speculative push/pop execution
  // -----------------------------------------------------------------------
  when(io.spec.pushValid) {
    speculativePush(io.spec.pushAddr, specStackPtr, specCounter,
      specReadPtr, specWritePtr, topEntry)
  }
  when(io.spec.popValid) {
    speculativePop(specStackPtr, specCounter,
      specReadPtr, specWritePtr, topNextPtr)
  }

  // -----------------------------------------------------------------------
  // Output: prediction address (predictedTop)
  // -----------------------------------------------------------------------
  io.spec.popAddr := predictedTop.returnAddr

  // Output: metadata for FTQ snapshot
  io.meta.specWritePtr := specWritePtr
  io.meta.specReadPtr  := specReadPtr
  io.meta.nextOnStack  := topNextPtr
  io.meta.specStackPtr := specStackPtr
  io.meta.specCounter  := specCounter

  // -----------------------------------------------------------------------
  // Commit logic: update commit stack at retirement
  // -----------------------------------------------------------------------
  private val commitTop = committedReturnAddressStack(committedReturnAddressStackPtr)

  when(io.commit.valid && io.commit.bits.isRet) {
    val ptrUpdate = Wire(UInt(log2Up(rasCommitStackSize).W))
    when(io.commit.bits.metaStackPtr =/= committedReturnAddressStackPtr) {
      ptrUpdate := io.commit.bits.metaStackPtr
    }.otherwise {
      ptrUpdate := committedReturnAddressStackPtr
    }
    when(commitTop.counter > 0.U) {
      committedReturnAddressStack(ptrUpdate).counter := commitTop.counter - 1.U
      committedReturnAddressStackPtr := ptrUpdate
    }.otherwise {
      committedReturnAddressStackPtr := decCommitStackPtr(ptrUpdate)
    }
  }

  private val commitPushAddr = speculativeReturnAddressQueue(io.commit.bits.metaWritePtr.value).returnAddr

  when(io.commit.valid && io.commit.bits.isCall) {
    val ptrUpdate = Wire(UInt(log2Up(rasCommitStackSize).W))
    when(io.commit.bits.metaStackPtr =/= committedReturnAddressStackPtr) {
      ptrUpdate := io.commit.bits.metaStackPtr
    }.otherwise {
      ptrUpdate := committedReturnAddressStackPtr
    }
    when(commitTop.counter < counterMax.U &&
         commitTop.returnAddr === commitPushAddr) {
      committedReturnAddressStack(ptrUpdate).counter := commitTop.counter + 1.U
      committedReturnAddressStackPtr := ptrUpdate
    }.otherwise {
      committedReturnAddressStackPtr := incCommitStackPtr(ptrUpdate)
      committedReturnAddressStack(incCommitStackPtr(ptrUpdate)).returnAddr := commitPushAddr
      committedReturnAddressStack(incCommitStackPtr(ptrUpdate)).counter    := 0.U
    }
  }

  // Advance specBasePtr on commit
  when(io.commit.valid && io.commit.bits.isCall) {
    specBasePtr := io.commit.bits.metaWritePtr
  }.elsewhen(io.commit.valid &&
    distanceBetween(io.commit.bits.metaWritePtr, specBasePtr) > 2.U) {
    specBasePtr := decQueuePtr(io.commit.bits.metaWritePtr)
  }

  // -----------------------------------------------------------------------
  // Redirect recovery: restore pointers, then redo call/ret
  // -----------------------------------------------------------------------
  when(io.redirect.valid) {
    specReadPtr  := io.redirect.meta.specReadPtr
    specWritePtr := io.redirect.meta.specWritePtr
    specStackPtr := io.redirect.meta.specStackPtr
    specCounter  := io.redirect.meta.specCounter

    when(io.redirect.isCall) {
      speculativePush(io.redirect.callAddr,
        io.redirect.meta.specStackPtr, io.redirect.meta.specCounter,
        io.redirect.meta.specReadPtr, io.redirect.meta.specWritePtr,
        redirectTopEntry)
    }
    when(io.redirect.isRet) {
      speculativePop(io.redirect.meta.specStackPtr,
        io.redirect.meta.specCounter,
        io.redirect.meta.specReadPtr, io.redirect.meta.specWritePtr,
        redirectNextPtr)
    }
  }

  // -----------------------------------------------------------------------
  // Overflow detection
  // -----------------------------------------------------------------------
  when(distanceBetween(specWritePtr, specBasePtr) > (rasSpecQueueSize - 2).U) {
    nearOverflow := true.B
  }.otherwise {
    nearOverflow := false.B
  }
  io.specNearOverflow := nearOverflow
}
