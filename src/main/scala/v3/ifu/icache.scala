//******************************************************************************
// Copyright (c) 2017 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// ICache
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.v3.ifu

import chisel3._
import chisel3.util._
import chisel3.util.random._

import org.chipsalliance.cde.config.{Parameters}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tile._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._
import freechips.rocketchip.util.property._
import freechips.rocketchip.rocket.{HasL1ICacheParameters, ICacheParams, ICacheErrors, ICacheReq}




import boom.v3.common._
import boom.v3.util.{BoomCoreStringPrefix}

/**
 * ICache module
 *
 * @param icacheParams parameters for the icache
 * @param hartId the id of the hardware thread in the cache
 * @param enableBlackBox use a blackbox icache
 */
class ICache(
  val icacheParams: ICacheParams,
  val staticIdForMetadataUseOnly: Int)(implicit p: Parameters)
  extends LazyModule
{
  lazy val module = new ICacheModule(this)
  val masterNode = TLClientNode(Seq(TLMasterPortParameters.v1(Seq(TLMasterParameters.v1(
    sourceId = IdRange(0, icacheParams.fetchMSHRNum),
    name = s"Core ${staticIdForMetadataUseOnly} ICache")))))

  val size = icacheParams.nSets * icacheParams.nWays * icacheParams.blockBytes
  private val wordBytes = icacheParams.fetchBytes
}

/**
 * IO Signals leaving the ICache
 *
 * @param outer top level ICache class
 */
class ICacheResp(val outer: ICache) extends Bundle
{
  val data = UInt((outer.icacheParams.fetchBytes*8).W)
  val replay = Bool()
  val ae = Bool()
}

/**
 * IO Signals for interacting with the ICache
 *
 * @param outer top level ICache class
 */
class ICacheBundle(val outer: ICache) extends BoomBundle()(outer.p)
  with HasBoomFrontendParameters
{
  val req = Flipped(Decoupled(new ICacheReq))
  val s1_paddr = Input(UInt(paddrBits.W)) // delayed one cycle w.r.t. req

  val s1_kill = Input(Bool()) // delayed one cycle w.r.t. req
  val s2_kill = Input(Bool()) // delayed two cycles; prevents I$ miss emission

  val resp = Valid(new ICacheResp(outer))
  val invalidate = Input(Bool())

  val perf = Output(new Bundle {
    val acquire = Bool()
  })

  //Enable_PerfCounter_Support
  val icache_valid_access = Output(Bool())
}

/**
 * Get a tile-specific property without breaking deduplication
 */
object GetPropertyByHartId
{
  def apply[T <: Data](tiles: Seq[RocketTileParams], f: RocketTileParams => Option[T], hartId: UInt): T = {
    PriorityMux(tiles.collect { case t if f(t).isDefined => (t.tileId.U === hartId) -> f(t).get })
  }
}

// Copied from XiangShan ICacheMissUnit.scala
class MSHRResp(implicit p: Parameters) extends CoreBundle with HasBoomFrontendParameters {
  val paddr  :  UInt = UInt(paddrBits.W)
  val tag:      UInt = UInt(tagBits.W)
  val vSetIdx:  UInt = UInt(idxBits.W)
  val way:      UInt = UInt(wayBits.W)
}

class LookUpMSHR(implicit p: Parameters) extends CoreBundle with HasBoomFrontendParameters {
  val paddr: UInt                = UInt(paddrBits.W)
  val hit:   Bool                = Input(Bool())
}

class ICacheMSHRIO(implicit p: Parameters) extends CoreBundle with HasBoomFrontendParameters {
  val fencei:    Bool                       = Input(Bool())
  val flush:     Bool                       = Input(Bool())
  val invalid:   Bool                       = Input(Bool())
  val req:       DecoupledIO[UInt]          = Flipped(DecoupledIO(UInt(paddrBits.W)))
  val acquire:   DecoupledIO[UInt]          = DecoupledIO(UInt(paddrBits.W))
  val lookUps:   LookUpMSHR                 = Flipped(new LookUpMSHR)
  val resp:      Valid[MSHRResp]            = ValidIO(new MSHRResp)
  val victimWay: UInt                       = Input(UInt(wayBits.W))

  val addr_valid_debug: Bool = Output(Bool())
}

// MSHR 与 flush 的交互
//   * 在 MSHR 发起 l2 请求之前，发生 flush，是否要 invalid 该 MSHR，在请求 fire 的同一周期呢？
//   * 在 l2 请求 fire 了之后，发生 flush，是否要 invalid 该写回？
// BOOM 的行为
//   * 忽略重定向造成的 flush，接受 fence.i 造成的 flush
//   * 在 MSHR 发起 l2 请求之前或请求 fire 的同一周期，忽略拉高的 fence.i 信号
//   * 在 l2 请求 fire 之后的 flush 信号会 invalid 后续写回（即使最后一个写回周期和 fence.i 同时拉高）
// 新引入的 MSHR 尽可能与 BOOM 保持一致：
//   * 在 fence.i 拉高时，prefetch MSHR 和 fetch MSHR 都会被 invalid。如果 l2 请求已经 fire 了,
//     会 invalid 后续写回
//   * fetch MSHR 不响应重定向造成的 flush
//   * 如果重定向造成的 flush 在 prefetch MSHR 发起 l2 请求之前或请求 fire 的同一周期, invalid 该 MSHR,
//     否则忽略该 flush 信号（因为对应的 cache line 大概率已经 invalid 掉了，写回不亏）

// 时序上：
//   * 预期的时序：s2 周期检查 mshr hit，入 mshr 的同时就可以发起 l2 请求（即 flow 为 true）
//   * mshr 处于 invalid 状态时才接受新请求（即 pipe 为 false）

// 冒险：在 s1 周期和 s2 周期我们都需要检查 MSHR hit，避免出现向下级 cache 请求已经写回的 cache line
// s1 周期我们检查 refill_done，看它与 io.s1_paddr 是否匹配，如果匹配则认为 MSHR hit
// s2 周期我们检查每个 MSHR 的 paddr 与 s2_paddr 是否匹配，如果匹配则认为 MSHR hit。如果 flush 或
// fence.i 同时拉高呢？这也无所谓，首先 fetch MSHR 不响应 flush，而 prefetch MSHR 遇到 flush 的话，
// prefetch 流水线应该同步清空。而 fence.i 本身就会 invalidate MSHR（这里的关键在于，误判为 MSHR hit
// 是安全的，因为 fetch 这边会 replay，重复请求。而误判为 MSHR miss 会导致对同一个 cache line 发起多
// 个 MSHR 请求，导致 ICache 中包含重复表项，这是危险的）

// TODO: 目前的实现是，当 isFetch 为 true 时，行为与 BOOM 保持一致；当 isFetch 为 false 时，接受 fence.i 和 flush 信号，
// 但是 flush 也会 invalidate 写回
class ICacheMSHR(isFetch: Boolean, ID: Int)(implicit p: Parameters) extends BoomModule
  with HasBoomFrontendParameters
{
  val io: ICacheMSHRIO = IO(new ICacheMSHRIO)

  private val valid = RegInit(Bool(), false.B)
  // this MSHR doesn't respond to fetch and sram
  private val flush  = RegInit(Bool(), false.B)
  private val fencei = RegInit(Bool(), false.B)
  // this MSHR has been issued
  private val issue = RegInit(Bool(), false.B)

  private val blkPaddr = RegInit(UInt((paddrBits - blockOffBits).W), 0.U)
  private val way      = RegInit(UInt(wayBits.W), 0.U)

  // look up and return result at the same cycle
  if (isFetch) {
    io.lookUps.hit := valid && !fencei && (io.lookUps.paddr(paddrBits-1, blockOffBits) === blkPaddr)
  } else {
    io.lookUps.hit := valid && !fencei && (io.lookUps.paddr(paddrBits-1, blockOffBits) === blkPaddr)
  }

  // invalid when the req hasn't been issued
  if (isFetch) {
    when (io.fencei) {
      when(!issue) {
        valid := false.B
      } .otherwise {
        fencei := true.B
      }
    }
  } else {
    when(io.fencei || io.flush) {
      when(!issue) {
        valid := false.B
      } .otherwise {
        fencei := true.B
        flush  := true.B
      }
    }
  }


  // receive request and register
  if (isFetch) {
    io.req.ready := !valid && !io.fencei
  } else {
    io.req.ready := !valid && !io.flush && !io.fencei
  }
  when(io.req.fire) {
    valid    := true.B
    flush    := false.B
    issue    := false.B
    fencei   := false.B
    blkPaddr := io.req.bits(paddrBits-1, blockOffBits)
  }

  // send request to L2
  if (isFetch) {
    io.acquire.valid := (valid || io.req.fire) && !issue && !io.fencei
  } else {
    io.acquire.valid := (valid || io.req.fire) && !issue && !io.flush && !io.fencei
  }
  io.acquire.bits := Cat(Mux(valid, blkPaddr, io.req.bits(paddrBits-1, blockOffBits)), 0.U(blockOffBits.W))

  // get victim way when acquire fire
  when(io.acquire.fire) {
    issue := true.B
    way   := io.victimWay
  }

  // invalid request when grant finish
  when(io.invalid) {
    valid  := false.B
    issue  := false.B
    fencei := false.B
    if (!isFetch) {
      flush  := false.B
    }
  }

  // offer the information other than data for write sram and response fetch
  if (isFetch) {
    io.resp.valid       := valid && !fencei
  } else {
    io.resp.valid         := valid && (!flush && !fencei)
  }
  io.resp.bits.paddr    := Cat(blkPaddr, 0.U(blockOffBits.W))
  io.resp.bits.tag      := blkPaddr(tagBits+idxBits-1,idxBits)
  io.resp.bits.vSetIdx  := blkPaddr(idxBits-1, 0)
  io.resp.bits.way      := way

  require(tagBits + idxBits == blkPaddr.getWidth)

  io.addr_valid_debug := issue
  if (IN_SIMULATION) {
    when (io.invalid) {
      assert (valid, p"ICache invalidating an invalid MSHR (ID = $ID)\n")
      assert (issue, p"ICache invalidating a non-issued MSHR (ID = $ID)\n")
    }
    when (!valid) {
      assert (!issue, p"ICache MSHR (ID = $ID) is not valid but has been issued\n")
      assert (!fencei, p"ICache MSHR (ID = $ID) is not valid but has fencei set\n")
    }
    when (!issue) {
      assert (!fencei, p"ICache MSHR (ID = $ID) is not issued but has fencei set\n")
    }
  }
}
// TODO: 
//   * 检查 MSHR 写回和流水线读 Tag Array 的冒险，避免为已经 hit 的 cache line 再发起 l2 请求
//   * 检查 fence.i 会 invalid 所有的 in-flight 读写和 cache line
// TODO: 校验 prefetch MSHR 的实现, 尤其是重定向的 flush，我觉得 s1 flush，s2 flush 和 MSHR flush
// 需要分别看待

/**
 * Main ICache module
 *
 * @param outer top level ICache class
 */
class ICacheModule(outer: ICache) extends LazyModuleImp(outer)
  with HasBoomCoreParameters
  with HasBoomFrontendParameters
{
  val enableICacheDelay = tileParams.core.asInstanceOf[BoomCoreParams].enableICacheDelay
  val io = IO(new ICacheBundle(outer))
  val (tl_out, edge_out) = outer.masterNode.out(0)

  require(isPow2(nSets) && isPow2(nWays))
  require(usingVM)
  require(pgIdxBits >= untagBits)

  // How many bits do we intend to fetch at most every cycle?
  val wordBits = outer.icacheParams.fetchBytes*8
  // Each of these cases require some special-case handling.
  require (tl_out.d.bits.data.getWidth == wordBits || (2*tl_out.d.bits.data.getWidth == wordBits && nBanks == 2))
  // If TL refill is half the wordBits size and we have two banks, then the
  // refill writes to only one bank per cycle (instead of across two banks every
  // cycle).
  val refillsToOneBank = (2*tl_out.d.bits.data.getWidth == wordBits)



  val s0_valid = io.req.fire
  val s0_vaddr = io.req.bits.addr

  val s1_valid = RegNext(s0_valid)
  val s1_tag_hit = Wire(Vec(nWays, Bool()))
  val s1_hit = s1_tag_hit.reduce(_||_)
  val s2_valid = RegNext(s1_valid && !io.s1_kill)
  val s2_hit = RegNext(s1_hit)
  val s2_paddr = RegNext(io.s1_paddr)

  val nFetchMSHRs = outer.icacheParams.fetchMSHRNum
  val fetchMSHRs = Seq.tabulate(nFetchMSHRs) { i =>
    Module(new ICacheMSHR(isFetch = true, ID = i))
  }

  val s1_MSHR_hit = Wire(Bool())
  val s2_MSHR_hit_vec = VecInit(fetchMSHRs.map(_.io.lookUps.hit))
  val s2_MSHR_hit = s2_MSHR_hit_vec.asUInt.orR
  val access_hit = s2_hit || s2_MSHR_hit || RegNext(s1_MSHR_hit)

  // Connect common control signals to all fetch MSHRs
  fetchMSHRs.foreach { m =>
    m.io.fencei := io.invalidate
    m.io.flush  := false.B
    m.io.lookUps.paddr := s2_paddr
  }

  // Allocate miss requests into the first available MSHR
  val need_mshr = s2_valid && !access_hit && !io.s2_kill
  var alloc_taken = false.B
  for (i <- 0 until nFetchMSHRs) {
    val can_alloc = need_mshr && fetchMSHRs(i).io.req.ready && !alloc_taken
    fetchMSHRs(i).io.req.valid := can_alloc
    fetchMSHRs(i).io.req.bits  := s2_paddr
    alloc_taken = alloc_taken || can_alloc
  }

  val mshr_resp_valids = VecInit(fetchMSHRs.map(_.io.resp.valid))
  val mshr_resp_paddrs = VecInit(fetchMSHRs.map(_.io.resp.bits.paddr))
  val mshr_resp_tags   = VecInit(fetchMSHRs.map(_.io.resp.bits.tag))
  val mshr_resp_idxs   = VecInit(fetchMSHRs.map(_.io.resp.bits.vSetIdx))
  val mshr_resp_ways   = VecInit(fetchMSHRs.map(_.io.resp.bits.way))

  val refill_fire = tl_out.a.fire
  val refill_one_beat = tl_out.d.fire && edge_out.hasData(tl_out.d.bits)
  val (_, _, d_done, refill_cnt) = edge_out.count(tl_out.d)
  val refill_done = refill_one_beat && d_done

  val refill_src = tl_out.d.bits.source
  val refill_src_oh = UIntToOH(refill_src, nFetchMSHRs)

  val refill_resp_valid = Mux1H(refill_src_oh, mshr_resp_valids)
  val invalidated = !refill_resp_valid
  val refill_paddr = Mux1H(refill_src_oh, mshr_resp_paddrs)
  val refill_tag   = Mux1H(refill_src_oh, mshr_resp_tags)
  val refill_idx   = Mux1H(refill_src_oh, mshr_resp_idxs)

  io.req.ready := !refill_one_beat
  //Enable_PerfCounter_Support
  io.icache_valid_access := s2_valid

  tl_out.d.ready := true.B
  require (edge_out.manager.minLatency > 0)

  val initial_fire = RegInit(true.B)
  initial_fire := false.B
  val victim_way = if (isDM) 0.U else LFSR(16, refill_fire || initial_fire)(log2Ceil(nWays)-1,0)
  val repl_way = Mux1H(refill_src_oh, mshr_resp_ways)
  fetchMSHRs.foreach(_.io.victimWay := victim_way)
  for (i <- 0 until nFetchMSHRs) {
    fetchMSHRs(i).io.invalid := refill_done && (refill_src === i.U)
  }
  s1_MSHR_hit := refill_done && !invalidated && refill_paddr(paddrBits-1, blockOffBits) ===
                  io.s1_paddr(paddrBits-1, blockOffBits)

  val tag_array = SyncReadMem(nSets, Vec(nWays, UInt(tagBits.W)))
  val tag_rdata = tag_array.read(s0_vaddr(untagBits-1, blockOffBits), !refill_done && s0_valid)
  when (refill_done) {
    tag_array.write(refill_idx, VecInit(Seq.fill(nWays)(refill_tag)), Seq.tabulate(nWays)(repl_way === _.U))
  }

  val vb_array = RegInit(0.U((nSets*nWays).W))
  when (refill_one_beat) {
    vb_array := vb_array.bitSet(Cat(repl_way, refill_idx), refill_done && !invalidated)
  }

  when (io.invalidate) {
    vb_array := 0.U
  }

  val s2_dout   = Wire(Vec(nWays, UInt(wordBits.W)))
  val s1_bankid = Wire(Bool())

  val s1_tag_only_hit = Wire(Vec(nWays, Bool()))
  val s1_vb_hit = Wire(Vec(nWays, Bool()))

  for (i <- 0 until nWays) {
    val s1_idx = io.s1_paddr(untagBits-1,blockOffBits)
    val s1_tag = io.s1_paddr(tagBits+untagBits-1,untagBits)
    s1_vb_hit(i) := vb_array(Cat(i.U, s1_idx))
    val tag = tag_rdata(i)
    s1_tag_only_hit(i) := tag === s1_tag
    s1_tag_hit(i) := s1_vb_hit(i) && s1_tag_only_hit(i)
  }

  dontTouch(s1_tag_only_hit)
  dontTouch(s1_vb_hit)
  dontTouch(s1_tag_hit)

  val ramDepth = if (refillsToOneBank && nBanks == 2) {
    nSets * refillCycles / 2
  } else {
    nSets * refillCycles
  }

  val dataArrays = if (nBanks == 1) {
    // Use unbanked icache for narrow accesses.
    (0 until nWays).map { x =>
      DescribedSRAM(
        name = s"dataArrayWay_${x}",
        desc = "ICache Data Array",
        size = ramDepth,
        data = UInt((wordBits).W)
      )
    }
  } else {
    // Use two banks, interleaved.
    (0 until nWays).map { x =>
      DescribedSRAM(
        name = s"dataArrayB0Way_${x}",
        desc = "ICache Data Array",
        size = ramDepth,
        data = UInt((wordBits/nBanks).W)
      )} ++
    (0 until nWays).map { x =>
      DescribedSRAM(
        name = s"dataArrayB1Way_${x}",
        desc = "ICache Data Array",
        size = ramDepth,
        data = UInt((wordBits/nBanks).W)
      )}
  }
  if (nBanks == 1) {
    // Use unbanked icache for narrow accesses.
    s1_bankid := 0.U
    for ((dataArray, i) <- dataArrays.zipWithIndex) {
      def row(addr: UInt) = addr(untagBits-1, blockOffBits-log2Ceil(refillCycles))
      val s0_ren = s0_valid

      val wen = (refill_one_beat && !invalidated) && repl_way === i.U

      val mem_idx = Mux(refill_one_beat, (refill_idx << log2Ceil(refillCycles)) | refill_cnt,
                    row(s0_vaddr))
      when (wen) {
        dataArray.write(mem_idx, tl_out.d.bits.data)
      }
      if (enableICacheDelay)
        s2_dout(i) := dataArray.read(RegNext(mem_idx), RegNext(!wen && s0_ren))
      else
        s2_dout(i) := RegNext(dataArray.read(mem_idx, !wen && s0_ren))
    }
  } else {
    // Use two banks, interleaved.
    val dataArraysB0 = dataArrays.take(nWays)
    val dataArraysB1 = dataArrays.drop(nWays)
    require (nBanks == 2)

    // Bank0 row's id wraps around if Bank1 is the starting bank.
    def b0Row(addr: UInt) =
      if (refillsToOneBank) {
        addr(untagBits-1, blockOffBits-log2Ceil(refillCycles)+1) + bank(addr)
      } else {
        addr(untagBits-1, blockOffBits-log2Ceil(refillCycles)) + bank(addr)
      }
    // Bank1 row's id stays the same regardless of which Bank has the fetch address.
    def b1Row(addr: UInt) =
      if (refillsToOneBank) {
        addr(untagBits-1, blockOffBits-log2Ceil(refillCycles)+1)
      } else {
        addr(untagBits-1, blockOffBits-log2Ceil(refillCycles))
      }

    s1_bankid := RegNext(bank(s0_vaddr))

    for (i <- 0 until nWays) {
      val s0_ren = s0_valid
      val wen = (refill_one_beat && !invalidated)&& repl_way === i.U

      var mem_idx0: UInt = null
      var mem_idx1: UInt = null

      if (refillsToOneBank) {
        // write a refill beat across only one beat.
        mem_idx0 =
          Mux(refill_one_beat, (refill_idx << (log2Ceil(refillCycles)-1)) | (refill_cnt >> 1.U),
          b0Row(s0_vaddr))
        mem_idx1 =
          Mux(refill_one_beat, (refill_idx << (log2Ceil(refillCycles)-1)) | (refill_cnt >> 1.U),
          b1Row(s0_vaddr))

        when (wen && refill_cnt(0) === 0.U) {
          dataArraysB0(i).write(mem_idx0, tl_out.d.bits.data)
        }
        when (wen && refill_cnt(0) === 1.U) {
          dataArraysB1(i).write(mem_idx1, tl_out.d.bits.data)
        }
      } else {
        // write a refill beat across both banks.
        mem_idx0 =
          Mux(refill_one_beat, (refill_idx << log2Ceil(refillCycles)) | refill_cnt,
          b0Row(s0_vaddr))
        mem_idx1 =
          Mux(refill_one_beat, (refill_idx << log2Ceil(refillCycles)) | refill_cnt,
          b1Row(s0_vaddr))

        when (wen) {
          val data = tl_out.d.bits.data
          dataArraysB0(i).write(mem_idx0, data(wordBits/2-1, 0))
          dataArraysB1(i).write(mem_idx1, data(wordBits-1, wordBits/2))
        }
      }
      if (enableICacheDelay) {
        s2_dout(i) := Cat(dataArraysB1(i).read(RegNext(mem_idx1), RegNext(!wen && s0_ren)),
                          dataArraysB0(i).read(RegNext(mem_idx0), RegNext(!wen && s0_ren)))
      } else {
        s2_dout(i) := RegNext(Cat(dataArraysB1(i).read(mem_idx1, !wen && s0_ren),
                                  dataArraysB0(i).read(mem_idx0, !wen && s0_ren)))
      }
    }
  }
  val s2_tag_hit = RegNext(s1_tag_hit)
  val s2_hit_way = OHToUInt(s2_tag_hit)
  val s2_bankid = RegNext(s1_bankid)
  val s2_way_mux = Mux1H(s2_tag_hit, s2_dout)

  val s2_unbanked_data = s2_way_mux
  val sz = s2_way_mux.getWidth
  val s2_bank0_data = s2_way_mux(sz/2-1,0)
  val s2_bank1_data = s2_way_mux(sz-1,sz/2)

  val s2_data =
    if (nBanks == 2) {
      Mux(s2_bankid,
        Cat(s2_bank0_data, s2_bank1_data),
        Cat(s2_bank1_data, s2_bank0_data))
    } else {
      s2_unbanked_data
    }

  io.resp.bits.ae := DontCare
  io.resp.bits.replay := DontCare
  io.resp.bits.data := s2_data
  io.resp.valid := s2_valid && s2_hit
  // TL-A: arbitrate among fetch MSHRs and encode MSHR ID into source field
  val acq_arb = Module(new RRArbiter(UInt(paddrBits.W), nFetchMSHRs))
  for (i <- 0 until nFetchMSHRs) {
    acq_arb.io.in(i) <> fetchMSHRs(i).io.acquire
  }

  tl_out.a.valid := acq_arb.io.out.valid
  acq_arb.io.out.ready := tl_out.a.ready

  val acq_source = acq_arb.io.chosen
  tl_out.a.bits := edge_out.Get(
    fromSource = acq_source,
    toAddress = acq_arb.io.out.bits,
    lgSize = lgCacheBlockBytes.U)._2
  tl_out.b.ready := true.B
  tl_out.c.valid := false.B
  tl_out.e.valid := false.B

  io.perf.acquire := tl_out.a.fire

  // Printf
  if (IN_SIMULATION) {
    // free-running cycle counter for simulation
    val (cycleCount, _) = Counter(true.B, Int.MaxValue)

    // plusarg to enable ICache debug prints
    val icache_printf = PlusArg("icache-printf", 0, "print icache debug info", 1)

    when (icache_printf(0)) {
      // 1) ICache sends miss request to lower level (TL-A channel)
      when (tl_out.a.fire) {
        val req_paddr = (refill_paddr >> blockOffBits) << blockOffBits
        printf(p"[${cycleCount} ICache Req] mshr=${acq_source} paddr=${Hexadecimal(req_paddr)}\n")
      }

      // 2) Cache line refill completes
      when (refill_done) {
        val line_paddr = (refill_paddr >> blockOffBits) << blockOffBits
        printf(p"[${cycleCount} ICache Refill Done] mshr=${refill_src} paddr=${Hexadecimal(line_paddr)} " +
          p"set=${Hexadecimal(refill_idx)} way=${Hexadecimal(repl_way)}, cancelled=${invalidated}\n")
      }

      // 3) Tag hit in ICache
      when (s1_valid && s1_hit && PopCount(s1_tag_hit) > 1.U) {
        printf(p"[${cycleCount} ICache Hit] paddr=${Hexadecimal(io.s1_paddr)} s1_tag_hit=${Binary(s1_tag_hit.asUInt)}\n")
      }
    }
  }

  assert(PopCount(s1_tag_hit) <= 1.U || !s1_valid || io.s1_kill,
    "Multiple ICache ways hit in s1")
  assert(!s1_valid || io.s1_kill || io.s1_paddr(pgIdxBits-1,0) === RegNext(io.req.bits.addr(pgIdxBits-1,0)),
    "s1_paddr does not match request address")
  val mshr_addr_valids = VecInit(fetchMSHRs.map(_.io.addr_valid_debug))
  val cur_mshr_addr_valid = Mux1H(refill_src_oh, mshr_addr_valids)
  assert(!refill_one_beat || cur_mshr_addr_valid,
    "ICache refill without a valid MSHR")

  if (IN_SIMULATION) {
    // Assertions on TL-D source range and non-interleaving between MSHRs.
    val d_inflight = RegInit(false.B)
    val d_source_reg = Reg(UInt(tl_out.d.bits.source.getWidth.W))
    // Track previous resp.valid value for the in-flight line's MSHR so that
    // we can detect false->true transitions while the line is in flight
    // (true->false is allowed for fence.i / flush).
    val d_prev_resp_valid = RegInit(false.B)

    when (refill_one_beat) {
      // 1) TL-D source must be within valid fetch MSHR ID range.
      assert(refill_src < nFetchMSHRs.U,
        "ICache TL-D source out of range of fetch MSHRs")

      // 2) While a line refill is in flight, all data beats must
      //    come from the same source until refill_done is true.
      when (!d_inflight) {
        d_inflight := true.B
        d_source_reg := refill_src
        d_prev_resp_valid := refill_resp_valid
      } .otherwise {
        assert(refill_src === d_source_reg,
          "ICache TL-D source interleaving between MSHRs within a single line refill")

        // 3) For the MSHR corresponding to this source, resp.valid must not
        //    switch from false to true while this line refill is in flight.
        assert(!(d_prev_resp_valid === false.B && refill_resp_valid === true.B),
          "ICache MSHR resp.valid changed from false to true during an in-flight line refill")

        d_prev_resp_valid := refill_resp_valid
      }

      when (refill_done) {
        d_inflight := false.B
      }
    }
  }

  override def toString: String = BoomCoreStringPrefix(
    "==L1-ICache==",
    "Fetch bytes   : " + cacheParams.fetchBytes,
    "Block bytes   : " + (1 << blockOffBits),
    "vaddr bits    : " + vaddrBits,
    "vaddr ext bits: " + vaddrBitsExtended,
    "min pg level  : " + minPgLevels,
    "pg level      : " + pgLevels,
    "Row bytes     : " + rowBytes,
    "Word bits     : " + wordBits,
    "Sets          : " + nSets,
    "Ways          : " + nWays,
    "Refill cycles : " + refillCycles,
    "RAMs          : (" +  wordBits/nBanks + " x " + nSets*refillCycles + ") using " + nBanks + " banks",
    "" + (if (nBanks == 2) "Dual-banked" else "Single-banked"),
    "I-TLB ways    : " + cacheParams.nTLBWays + "\n")
}


