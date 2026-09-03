package boom.v3.ifu

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import boom.v3.common._

class BLBPBranchPredictorBank(implicit p: Parameters) extends BranchPredictorBank()(p) {
  // Stage 3a: inert pass-through — io.resp := io.resp_in(0) inherited.
  // Stage 3b: ITC population + candidate-pool observer.
  // Stage 4a: predict-side ITC read + 4 observer counters.
  // Stage 4b: fingerprint compute datapath (observer-only).
  // Stage 4c: training + f3_meta carry + convergence observer.

  def itc_nSets = p(BoomBlbpITCSets)
  def itc_nWays = p(BoomBlbpITCWays)
  def override_thresh = p(BoomBlbpOverrideThresh)
  val useRRIP = p(BoomBlbpUseRRIP)
  val usePLRU = p(BoomBlbpUsePLRU)
  val useDotProduct = p(BoomBlbpUseDotProduct)
  val useTransfer = p(BoomBlbpUseTransfer)
  val useAdaptive = p(BoomBlbpUseAdaptive)
  val useSelectiveBit = p(BoomBlbpUseSelectiveBit)
  val useTargetHash = p(BoomBlbpUseTargetHash)
  val useTags = p(BoomBlbpUseTags)
  val blbpTagBits = p(BoomBlbpTagBits)
  val useRegion  = p(BoomBlbpUseRegion)
  val lg2Regions = p(BoomBlbpLg2Regions)
  val offsetBits = p(BoomBlbpOffsetBits)
  val nRegions   = p(BoomBlbpNRegions)
  val regionRRIP = p(BoomBlbpRegionRRIP)
  require(isPow2(nRegions), s"BoomBlbpNRegions ($nRegions) must be a power of 2")
  require(!(usePLRU && useRRIP), "PLRU and RRIP are mutually exclusive")
  val regionIdxW = log2Ceil(nRegions)
  val regionBits = vaddrBitsExtended - offsetBits
  val tgtBits    = if (useRegion) regionIdxW + offsetBits else vaddrBitsExtended
  val usePrivateHist = p(BoomBlbpUsePrivateHist)
  val useLocalHist   = p(BoomBlbpUseLocalHist)
  val nLHist         = p(BoomBlbpNLHist)
  val lhLength       = p(BoomBlbpLHLength)
  val lbit           = p(BoomBlbpLBit)
  val histIdBits     = p(BoomBlbpHistIdBits)
  val thetaInit   = p(BoomBlbpThetaInit)
  val thetaStep   = p(BoomBlbpThetaStep)
  val thetaSpeed  = p(BoomBlbpThetaSpeed)
  val THETA_W = 20   // |sum| reaches ~16 bits with transfer; +headroom for theta

  // Convex, monotonic transfer on |weight|. 5-bit signed weights => |w| in 0..16,
  // so 17 entries. xlat[0] = 0 (zero weight contributes nothing). Increasing
  // first differences => convex (amplifies confident weights). SPEC-tunable.
  val xlatRom = VecInit(
    Seq(0, 1, 2, 4, 6, 8, 11, 14, 18, 22, 27, 32, 38, 44, 51, 58, 66).map(_.S(8.W)))
  // val xlatRom = VecInit(Seq.fill(17)(0.S(8.W)))   // PROBE ONLY — revert after

  // Read-side magnitude transfer. Pure function => identical HW at every call site.
  def xfer(wt: SInt): SInt = {
    if (!useTransfer) {
      wt
    } else {
      val mag = Mux(wt < 0.S, -wt, wt).asUInt   // 0..16
      val m   = xlatRom(mag)                      // SInt(8.W), >= 0
      Mux(wt < 0.S, -m, m)
    }
  }

  class ITCEntry extends Bundle {
    val valid  = Bool()
    val tag    = UInt(blbpTagBits.W)
    val tgt    = UInt(tgtBits.W)   // full target (off) or {region_index, offset} (on)
    val rpv    = UInt(2.W)
  }

  // Folded Cat(set,col) index — full-entry write (no mask) → BRAM-compatible
  val itc = Seq.fill(itc_nWays) { SyncReadMem(itc_nSets * bankWidth, new ITCEntry) }
  def itcAddr(set: UInt, col: UInt): UInt = Cat(set, col(log2Ceil(bankWidth)-1, 0))

  // Region table: compressed {region_index, offset} → full target reconstruction
  val region_entries = Mem(nRegions, UInt(regionBits.W))
  val region_valid   = RegInit(VecInit(Seq.fill(nRegions)(false.B)))
  // RRIP replacement for region table (2-bit RRPV, opt-in; default random unchanged)
  val region_rrpv = if (regionRRIP) Some(RegInit(VecInit(Seq.fill(nRegions)(3.U(2.W))))) else None
  val rand_counter   = RegInit("hdeadb10c".U(32.W))

  val region_init_done = RegInit(false.B)
  val region_init_idx  = RegInit(0.U(log2Ceil(nRegions).W))

  when (!region_init_done) {
    region_entries.write(region_init_idx, 0.U)
    region_init_idx := region_init_idx + 1.U
    when (region_init_idx === (nRegions - 1).U) {
      region_init_done := true.B
    }
  }

  def entryTarget(e: ITCEntry): UInt =
    if (!useRegion) e.tgt
    else (region_entries.read(e.tgt(tgtBits-1, offsetBits)) << offsetBits) |
          e.tgt(offsetBits-1, 0)

  // ── Fingerprint datapath constants ──────────────────────────────────────────
  val F = 15
  val Tbl = 8
  val E = 1024
  val nbias = 4096

  // Direction-history fold windows (capped by BOOM global history) — direction only
  val histLens = Seq(0, 2, 4, 8, 16, 16, 32, 64).map(_ min globalHistoryLength)

  // idbits history depth: config-driven, DECOUPLED from globalHistoryLength
  val maxHist = blbpIdhLen                       // was histLens.max; now the config knob (default 42)
  require(histIdBits == blbpIdhShift,
    s"histIdBits ($histIdBits) must equal blbpIdhShift ($blbpIdhShift) — set both via WithBlbpIdbits")
  require(histIdBits <= F, s"histIdBits ($histIdBits) cannot exceed fingerprint width F ($F)")

  // idbits fold windows: scale the base pattern to maxHist, NOT capped by globalHistoryLength.
  // At maxHist==42 (default) this is exactly the old shared histLens => behavior unchanged.
  private val idhLensBase = Seq(0, 2, 4, 8, 12, 18, 28, 42)
  val idhLens = idhLensBase.map(l => math.min(l * maxHist / idhLensBase.max, maxHist))
  require(idhLens.max <= maxHist && idhLens.length == Tbl,
    s"idhLens must have Tbl entries and max <= maxHist")

  // Per-branch local history: shift register of one target fingerprint bit
  val lhist = RegInit(VecInit(Seq.fill(nLHist)(0.U(lhLength.W))))
  def lhIdx(pc: UInt): UInt = {
    val fi = fetchIdx(pc); val w = log2Ceil(nLHist)
    (fi(w-1,0) ^ fi(2*w-1, w))
  }

  // Private mixed history: commit-updated, carried in meta (read-row == write-row).
  val blbp_ghist = if (!useBlbpSpecHist) RegInit(0.U(maxHist.W)) else 0.U(maxHist.W)
  val idhPredict = if (useBlbpSpecHist) io.f1_idh else blbp_ghist   // s1-aligned

  // ── MPP multi-perspective features (CBP2025 tuned subset) ─────────────────
  // Each weight table XORs one MPP feature into its ORIGINAL index
  // (pc ^ ghist/idh/lhist) as an extra aliasing-reduction source:
  //   t0 MODHIST(5,17) mod7 | t1 MODHIST(3,22) mod5
  //   t2 GHISTMODPATH(0,19,5) mod2 | t3 GHISTMODPATH(3,14,6) mod5
  //   t4 MODPATH(1,26,3) mod3 | t5 BACKPATH(22,6)
  //   t6 RECENCY(10,1) | t7 RECENCYPOS(56)
  // Registers update on jalr commits only (tr_fire); hashed at s0 for table indexing.

  // Modulo outcome registers (one per maintained modulus; newest bit at index 0)
  val modHist2 = RegInit(VecInit(Seq.fill(20)(false.B)))   // mod 2, len 20
  val modHist5 = RegInit(VecInit(Seq.fill(23)(false.B)))   // mod 5, len 23
  val modHist7 = RegInit(VecInit(Seq.fill(18)(false.B)))   // mod 7, len 18

  // Modulo path registers (16-bit pc2 entries, newest at index 0)
  val modPath2 = RegInit(VecInit(Seq.fill(20)(0.U(16.W)))) // mod 2, depth 20
  val modPath3 = RegInit(VecInit(Seq.fill(27)(0.U(16.W)))) // mod 3, depth 27
  val modPath5 = RegInit(VecInit(Seq.fill(15)(0.U(16.W)))) // mod 5, depth 15

  // Backward-branch path (only taken branches jumping backward are recorded)
  val backPath = RegInit(VecInit(Seq.fill(22)(0.U(16.W)))) // depth 22

  // Recency stack (move-to-front, deduplicated recent pc2s)
  val recencyStack = RegInit(VecInit(Seq.fill(56)(0.U(16.W)))) // assoc 56

  def pc2(pc: UInt): UInt = pc(17, 2)                     // C++ pc >> 2 (16-bit)
  def pcHash16(pc: UInt): UInt =                          // house analog of hash_pc
    (pc(15, 0) ^ pc(31, 16) ^ pc(39, 32))(15, 0)

  // Fold x into chunkW-wide XOR chunks (10-bit chunks for the table index)
  def foldBits(x: UInt, chunkW: Int): UInt = {
    val nChunks = (x.getWidth + chunkW - 1) / chunkW
    val chunks = (0 until nChunks).map { i =>
      x(Math.min((i + 1) * chunkW, x.getWidth) - 1, i * chunkW)
    }
    chunks.reduce(_ ^ _)
  }

  // Shift-add hash (C++ x = (x<<shift) + entry): pre-shifted terms, tree-reduced.
  // Terms with shift*(n-1-i) >= bits are shifted out entirely (block size 30).
  def shiftAdd(entries: Seq[UInt], shift: Int, bits: Int = 30): UInt = {
    val n = entries.size
    val terms = (0 until n).map { i =>
      val k = shift * (n - 1 - i)
      if (k >= bits) 0.U(bits.W)
      else (entries(i) << k).pad(bits)(bits - 1, 0)   // pad: entries are only 16-17 bits
    }
    treeReduce(terms)(_ + _)
  }

  // Feature hash per table (matches the C++ hash_* helpers, block size 30)
  def mppFeatureHash(t: Int, pc: UInt): UInt = t match {
    case 0 => modHist7.asUInt                              // MODHIST(5,17) — identity fold (len < 30)
    case 1 => modHist5.asUInt                              // MODHIST(3,22)
    case 2 => shiftAdd((0 until 19).map(i => (modPath2(i) << 1) | modHist2(i)), 5)
    case 3 => shiftAdd((0 until 14).map(i => (modPath5(i) << 1) | modHist5(i)), 6)
    case 4 => shiftAdd((0 until 26).map(i => modPath3(i)), 3)
    case 5 => shiftAdd((0 until 22).map(i => backPath(i)), 6)
    case 6 => shiftAdd((0 until 10).map(i => recencyStack(i)), 1)
    case 7 => {                                            // RECENCYPOS(56)
      val match_oh = VecInit((0 until 56).map(i => recencyStack(i) === pc2(pc)))
      val hit = match_oh.asUInt.orR
      val pos = PriorityEncoder(match_oh)
      Mux(hit, (pos * E.U) / 56.U, (E - 1).U)
    }
  }

  // Feature hash folded to index width — XORed INTO the original table index
  // (pc/ghist/idh/lhist), adding the MPP source to reduce aliasing.
  def mppFold(t: Int, pc: UInt): UInt =
    foldBits(mppFeatureHash(t, pc), log2Ceil(E))

  val useBias   = p(BoomBlbpUseBias)
  val coeffBias = 68
  val coeffs = VecInit(Seq(68, 56, 48, 37, 31, 24, 17, 17).map(_.S(8.W)))

  // Per-bit adaptive threshold (Seznec/O-GEHL). Module Regs, commit-updated.
  val theta = RegInit(VecInit(Seq.fill(F)(thetaInit.S(THETA_W.W))))
  val tc    = RegInit(VecInit(Seq.fill(F)(0.S(8.W))))

  val adapt_train = WireDefault(false.B)

  val weights = Seq.fill(Tbl)(SyncReadMem(E, Vec(F, SInt(5.W))))
  val biasMem = if (useBias) Some(SyncReadMem(nbias, Vec(F, SInt(5.W)))) else None
  val fp_untrained = ((1 << F) - 1).U(F.W)

  // ── Meta layout constants (LSB-first) ─────────────────────────────────────
  // [W_FP-1:0] fp (15) | [W_SIG+W_FP-1:W_FP] sig (15) |
  // [W_HAM+W_SIG+W_FP-1:W_SIG+W_FP] min_ham (4) |
  // [W_BASE-1:W_HAM+W_SIG+W_FP] valid (1) → W_BASE = 35
  val W_FP = F; val W_SIG = F; val W_HAM = 4; val W_VALID = 1
  val W_BASE = W_FP + W_SIG + W_HAM + W_VALID  // 35
  val W_BIAS  = if (useBias) F * 5 else 0            // 75 or 0 (gated)
  val W_WT    = Tbl * F * 5                        // 600
  val W_ITC_ENTRY = 1 + blbpTagBits + tgtBits + 2   // valid(1) + tag + tgt + rpv(2)
  val W_POOL  = itc_nWays * W_ITC_ENTRY
  val offBias = W_BASE                             // 35
  val offWt   = offBias + W_BIAS                   // 110 (bias on) or 35 (bias off)
  val offPool = offWt   + W_WT                     // 710
  val W_HIST  = maxHist                              // carried private history
  val offHist = offPool + W_POOL                     // after pool
  val W_LHIST  = lhLength                             // carried local history
  val offLHist = offHist + W_HIST                     // after private hist
  // Attribution fields are appended above the existing layout so all current
  // low-bit fingerprint, pool, and history offsets remain stable.
  val offOverride   = offLHist + W_LHIST
  val offBaseValid  = offOverride + 1
  val offBaseTarget = offBaseValid + 1
  val offNewTarget  = offBaseTarget + vaddrBitsExtended

  override val metaSz = offNewTarget + vaddrBitsExtended
  require(metaSz <= bpdMaxMetaLength,
    s"BLBP metaSz ($metaSz) exceeds bpdMaxMetaLength ($bpdMaxMetaLength) " +
    s"— reduce itc_nWays ($itc_nWays) or increase bpdMaxMetaLength")

  println(s"[BLBP cfg] hash=$useTargetHash region=$useRegion dot=$useDotProduct " +
    s"xfer=$useTransfer adapt=$useAdaptive selbit=$useSelectiveBit ph=$usePrivateHist " +
    s"lh=$useLocalHist tags=$useTags bias=$useBias rrip=$useRRIP " +
    s"sets=$itc_nSets ways=$itc_nWays theta=$thetaInit/$thetaStep/$thetaSpeed " +
    s"maxHist=$maxHist metaSz=$metaSz/$bpdMaxMetaLength")

  println(s"[meta] offHist=$offHist W_HIST=$W_HIST offLHist=$offLHist offPool=$offPool W_POOL=$W_POOL metaSz=$metaSz")

  println(s"[BLBP mpp] t0 MODHIST(5,17) t1 MODHIST(3,22) t2 GHISTMODPATH(0,19,5) " +
    s"t3 GHISTMODPATH(3,14,6) t4 MODPATH(1,26,3) t5 BACKPATH(22,6) t6 RECENCY(10,1) t7 RECENCYPOS(56)")

  override val mems =
    Seq.tabulate(itc_nWays)(w => (s"blbp_itc_way$w", itc_nSets * bankWidth, 1 + blbpTagBits + tgtBits + 2)) ++
    Seq.tabulate(Tbl)(t => (s"blbp_weight$t", E, F * 5)) ++
    (if (useBias) Seq(("blbp_bias", nbias, F * 5)) else Nil)

  // Index helpers
  def foldHist(hist: UInt, len: Int): UInt = {
    if (len == 0) 0.U(log2Ceil(E).W)
    else {
      val h = hist(len - 1, 0)
      val chunkW = log2Ceil(E)
      val nChunks = (len + chunkW - 1) / chunkW
      val chunks = (0 until nChunks).map { i =>
        h(Math.min((i + 1) * chunkW, len) - 1, i * chunkW)
      }
      chunks.reduce(_ ^ _)
    }
  }
  def tblIdx(pc: UInt, ghist: UInt, t: Int): UInt =
    (fetchIdx(pc) ^ foldHist(ghist, histLens(t)))(log2Ceil(E) - 1, 0)

  // Hybrid: directions from `ghist`, indirect idbits from `idh`, XOR-mixed
  def tblIdxMix(pc: UInt, ghist: UInt, idh: UInt, t: Int): UInt =
    (fetchIdx(pc) ^ foldHist(ghist, histLens(t)) ^ foldHist(idh, idhLens(t)))(log2Ceil(E) - 1, 0)

  // Target fingerprint extraction. Bit-selection (default) or XOR-folded hash.
  def extractFp(target: UInt): UInt = {
    if (!useTargetHash) {
      val bits = Seq(1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 16, 17, 19)
      VecInit(bits.map(b => target(b))).asUInt
    } else {
      // XOR-fold the full target into F bits (all bits contribute)
      val tw = vaddrBitsExtended
      val nChunks = (tw + F - 1) / F
      val padded = Cat(0.U((nChunks * F - tw).W), target)
      (0 until nChunks).map(i => padded(i * F + F - 1, i * F)).reduce(_ ^ _)
    }
  }

  // ── Weight/bias init FSM ────────────────────────────────────────────────────
  val wt_init_done = RegInit(false.B)
  val wt_init_idx  = RegInit(0.U(log2Ceil(nbias).W))
  val INIT_VAL = 0.S(5.W)
  val initRow = Wire(Vec(F, SInt(5.W))); initRow := VecInit(Seq.fill(F)(INIT_VAL))

  when (!wt_init_done) {
    wt_init_idx := wt_init_idx + 1.U
    when (wt_init_idx === (nbias - 1).U) { wt_init_done := true.B }
  }

  // ── Fingerprint dot-product (s1→s3) ─────────────────────────────────────────
  // PREDICT-SIDE sum_w formula (for bit-identity check with training):
  //   s2_sum(w) = Σ_t (s2_wt(t)(w) * coeffs(t)) + (s2_bia(w) * coeffBias)
  //   s3_sum = RegNext(s2_sum); fp_bit = (s3_sum(w) >= 0.S)
  val biasIdx = if (useBias) fetchIdx(s1_pc)(log2Ceil(nbias) - 1, 0) else 0.U
  val readEn  = s1_valid && wt_init_done
  val local_hist = lhist(lhIdx(s1_pc))                 // combinational, s1
  // MPP feature folds: hashed at s0 (f0_pc + commit-updated feature regs),
  // registered to s1 and XORed into the original index — read timing unchanged.
  val s0_mpp_fold = VecInit((0 until Tbl).map(t => mppFold(t, io.f0_pc)))
  val s1_mpp_fold = RegNext(s0_mpp_fold)
  val s2_wt = VecInit((0 until Tbl).map { t =>
    val idx =
      if (useLocalHist && t == 0) (fetchIdx(s1_pc) ^ local_hist)(log2Ceil(E)-1, 0)
      else if (usePrivateHist)    tblIdxMix(s1_pc, io.f1_ghist, idhPredict, t)
      else                        tblIdx(s1_pc, io.f1_ghist, t)
    weights(t).read(idx ^ s1_mpp_fold(t), readEn)
  })
  val s2_bia = if (useBias) biasMem.get.read(biasIdx, readEn) else VecInit(Seq.fill(F)(0.S(5.W)))

  val s2_sum = VecInit((0 until F).map { w =>
    val wt = (0 until Tbl).map { t => (xfer(s2_wt(t)(w)) * coeffs(t)).asSInt }.reduce(_ + _)
    if (useBias) (wt + (xfer(s2_bia(w)) * coeffBias.S).asSInt).asSInt
    else         wt
  })

  val s3_sum = RegNext(s2_sum)
  val s3_bia = if (useBias) RegNext(s2_bia) else 0.U.asTypeOf(Vec(F, SInt(5.W)))
  val s3_wt  = RegNext(s2_wt)
  val s3_local_hist = RegNext(RegNext(local_hist))   // s1 → s3
  val s3_blbp_ghist =
    if (useBlbpSpecHist) RegNext(RegNext(io.f1_idh))   // s1 -> s3, matches idhPredict vintage
    else                 RegNext(RegNext(blbp_ghist))
  val s3_fingerprint = VecInit(s3_sum.map(s => (s >= 0.S).asUInt)).asUInt

  // ── Commit-side Training RMW ────────────────────────────────────────────────
  val u        = io.update.bits
  val tr_fire  = io.update.valid && u.is_commit_update &&
                 u.cfi_is_jalr && !u.cfi_is_ret && u.cfi_taken && u.cfi_idx.valid

  // Tag for commit-side hit detection + train-side pool (defined early for sb_valid)
  val b_tag = RegNext(itcTag(u.pc))

  // commit-side reads DROPPED — weights + ITC carried in meta (TAGE pattern)
  // Training index = original index recomputed at commit, XORed with the MPP
  // feature fold from the PRE-update registers (same vintage as the predict-time
  // read; register updates land at the same edge as tr_b_idx).
  def tblIdxUpd(t: Int): UInt = {
    val idx =
      if (useLocalHist && t == 0) (fetchIdx(u.pc) ^ carried_lhist)(log2Ceil(E)-1, 0)
      else if (usePrivateHist)    tblIdxMix(u.pc, io.update.bits.ghist, carried_hist, t)
      else                        tblIdx(u.pc, io.update.bits.ghist, t)
    idx ^ mppFold(t, u.pc)
  }

  // 1 cycle later: unpack carried snapshots
  val tr_b_fire      = RegNext(tr_fire, false.B)
  val tr_b_target    = RegNext(u.target)
  val tr_b_fp        = RegNext(io.update.bits.meta(F - 1, 0))
  val tr_carried_bia = if (useBias)
    RegNext(io.update.bits.meta(offBias + W_BIAS - 1, offBias)).asTypeOf(Vec(F, SInt(5.W)))
    else VecInit(Seq.fill(F)(0.S(5.W)))
  val tr_carried_wt  = RegNext(io.update.bits.meta(offWt   + W_WT   - 1, offWt  )).asTypeOf(Vec(Tbl, Vec(F, SInt(5.W))))
  val tr_carried_pool= RegNext(io.update.bits.meta(offPool + W_POOL - 1, offPool)).asTypeOf(Vec(itc_nWays, new ITCEntry))
  val carried_hist  = io.update.bits.meta(offHist  + W_HIST  - 1, offHist)
  val carried_lhist = io.update.bits.meta(offLHist + W_LHIST - 1, offLHist)

  // Commit-time private history update: indirect-only idbits (directions from shared ghist).
  // In speculative mode the shift lives in GlobalHistory.update; this runs only off.
  if (usePrivateHist && !useBlbpSpecHist) {
    val commitIndir = io.update.valid && u.is_commit_update &&
                      u.cfi_is_jalr && !u.cfi_is_ret && u.cfi_idx.valid
    when (commitIndir) {
      blbp_ghist := ((blbp_ghist << histIdBits) |
                     extractFp(u.target)(histIdBits - 1, 0))(maxHist - 1, 0)
    }
  }

  // Commit-time local history update: per-branch, one fingerprint bit per indirect
  if (useLocalHist) {
    val commitIndir = io.update.valid && u.is_commit_update &&
                      u.cfi_is_jalr && !u.cfi_is_ret && u.cfi_idx.valid
    val li = lhIdx(u.pc)
    when (commitIndir) {
      lhist(li) := (Cat(lhist(li), extractFp(u.target)(lbit)))(lhLength-1, 0)
    }
  }
  val tr_b_idx       = RegInit(VecInit(Seq.fill(Tbl)(0.U(log2Ceil(E).W))))
  val tr_b_bias_idx  = RegInit(0.U(log2Ceil(nbias).W))

  // MPP commit-side helpers (combinational; shared with the [mpp] diagnostics)
  val hpc_upd = pcHash16(u.pc)
  val p2_upd  = pc2(u.pc)
  val hashed_taken_upd = u.pc(2)     // tr_fire taken=1: taken ^ !(pc & (1<<2)) == pc(2)
  val rec_match_upd = VecInit((0 until 56).map(i => recencyStack(i) === p2_upd))
  val rec_hit_upd   = rec_match_upd.asUInt.orR
  val rec_pos_upd   = PriorityEncoder(rec_match_upd)

  when (tr_fire) {
    tr_b_idx      := VecInit((0 until Tbl).map(t => tblIdxUpd(t)))
    if (useBias) { tr_b_bias_idx := fetchIdx(u.pc)(log2Ceil(nbias) - 1, 0) }

    // ── MPP history register updates (jalr commit; C++ spec_update rules) ────
    // mod 2: modHist2 + modPath2 (GHISTMODPATH(0,19,5))
    when (hpc_upd % 2.U === 0.U) {
      modHist2(0) := hashed_taken_upd
      for (i <- 1 until 20) modHist2(i) := modHist2(i - 1)
      modPath2(0) := p2_upd
      for (i <- 1 until 20) modPath2(i) := modPath2(i - 1)
    }
    // mod 3: modPath3 (MODPATH(1,26,3))
    when (hpc_upd % 3.U === 0.U) {
      modPath3(0) := p2_upd
      for (i <- 1 until 27) modPath3(i) := modPath3(i - 1)
    }
    // mod 5: modHist5 (MODHIST(3,22), GHISTMODPATH(3,14,6)) + modPath5
    when (hpc_upd % 5.U === 0.U) {
      modHist5(0) := hashed_taken_upd
      for (i <- 1 until 23) modHist5(i) := modHist5(i - 1)
      modPath5(0) := p2_upd
      for (i <- 1 until 15) modPath5(i) := modPath5(i - 1)
    }
    // mod 7: modHist7 (MODHIST(5,17))
    when (hpc_upd % 7.U === 0.U) {
      modHist7(0) := hashed_taken_upd
      for (i <- 1 until 18) modHist7(i) := modHist7(i - 1)
    }
    // backward-branch path (taken jalr jumping backward — loop return edge)
    when (u.target < u.pc) {
      backPath(0) := p2_upd
      for (i <- 1 until 22) backPath(i) := backPath(i - 1)
    }
    // recency stack move-to-front (C++ insert_recency)
    recencyStack(0) := Mux(rec_hit_upd, recencyStack(rec_pos_upd), p2_upd)
    for (i <- 1 until 56) {
      recencyStack(i) := Mux(rec_hit_upd,
        Mux(rec_pos_upd >= i.U, recencyStack(i - 1), recencyStack(i)),
        recencyStack(i - 1))
    }
  }

  // ── MPP simulation diagnostics ─────────────────────────────────────────────
  if (IN_SIMULATION) {
    val mpp_cnt = RegInit(0.U(8.W))
    when (tr_fire && mpp_cnt < 32.U) {
      mpp_cnt := mpp_cnt + 1.U
      printf(p"[mpp] hpc=${hpc_upd} g2=${hpc_upd % 2.U} g3=${hpc_upd % 3.U} " +
        p"g5=${hpc_upd % 5.U} g7=${hpc_upd % 7.U} " +
        p"back=${u.target < u.pc} recHit=${rec_hit_upd} recPos=${rec_pos_upd} pc2=${p2_upd}\n")
    }
  }

  // Training RMW at tr_b_fire: one write per table, all F bits updated in parallel
  val tr_actual_fp = extractFp(tr_b_target)

  // Wire declarations must precede their use in the debug printf block below.
  // Per-bit train_we and updated rows (combinational, computed at tr_b_fire)
  val tr_we = Wire(Vec(F, Bool()))
  val tr_updated_wt = Wire(Vec(Tbl, Vec(F, SInt(5.W))))
  val tr_updated_bias = if (useBias) Wire(Vec(F, SInt(5.W))) else VecInit(Seq.fill(F)(0.S(5.W)))

  if (IN_SIMULATION && useAdaptive) {
    when (tr_fire) {
      val w0 = 0  // sample bit 0
      val sum_w0 = {
        val wsum = (0 until Tbl).map { t =>
          (xfer(tr_carried_wt(t)(w0)) * coeffs(t)).asSInt
        }.reduce(_ + _)
        if (useBias) wsum + (xfer(tr_carried_bia(w0)) * coeffBias.S).asSInt
        else         wsum
      }
      val pred_bit0   = (sum_w0 >= 0.S).asUInt
      val target_bit0 = tr_actual_fp(w0)
      printf(p"[adapt] sum0=${sum_w0} theta0=${theta(w0)} tc0=${tc(w0)} " +
        p"underConf=${sum_w0.abs < theta(w0)} we=${tr_we(w0)} correct=${pred_bit0 === target_bit0}\n")
    }
  }

  // 在 tr_fire 附近，不依赖 useAdaptive 的打印
  if (IN_SIMULATION) {
    when (tr_fire) {
      val w0 = 0
      val sum_w0 = {
        val wsum = (0 until Tbl).map { t =>
          (xfer(tr_carried_wt(t)(w0)) * coeffs(t)).asSInt
        }.reduce(_ + _)
        if (useBias) wsum + (xfer(tr_carried_bia(w0)) * coeffBias.S).asSInt
        else         wsum
      }
      printf(p"[sum0] val=${sum_w0}\n")
    }
  }

  // TRAINING-SIDE sum_w formula (must be bit-identical to predict-side s2_sum above):
  //   sum_w = Σ_t (tr_carried_wt(t)(w) * coeffs(t)) + (tr_carried_bia(w) * coeffBias)
  // (tr_we, tr_updated_wt, tr_updated_bias moved above the debug printf block)

  // Which-bits mask from carried pool (same pool s3_pool / cand_fp used at predict)
  val sb_fp    = VecInit(tr_carried_pool.map(e => extractFp(entryTarget(e))))
  val sb_valid = VecInit(tr_carried_pool.map(e =>
    e.valid && (if (useTags) (e.tag === b_tag) else true.B)))
  def discriminative(w: Int): Bool = {
    val anyOne  = (0 until itc_nWays).map(c => sb_valid(c) &&  sb_fp(c)(w)).reduce(_ || _)
    val anyZero = (0 until itc_nWays).map(c => sb_valid(c) && !sb_fp(c)(w)).reduce(_ || _)
    anyOne && anyZero
  }

  // Cold-start / single-candidate fallback: if NO bit is discriminative across the
  // commit-side pool (cold weights, or ≤1 distinct target), SelectiveBit would gate
  // training to zero forever. Mirror the C++ which_bits behavior: only suppress bits
  // when there is a real discriminative set to suppress against. Otherwise train all.
  val anyDiscrim = if (useSelectiveBit) (0 until F).map(w => discriminative(w)).reduce(_ || _) else true.B

  for (w <- 0 until F) {
    val sum_w = {
      val wsum = (0 until Tbl).map { t =>
        (xfer(tr_carried_wt(t)(w)) * coeffs(t)).asSInt
      }.reduce(_ + _)
      if (useBias) wsum + (xfer(tr_carried_bia(w)) * coeffBias.S).asSInt
      else         wsum
    }
    val pred_bit   = (sum_w >= 0.S).asUInt
    val target_bit = tr_actual_fp(w)
    val correct = (pred_bit === target_bit)
    val absSum  = Mux(sum_w < 0.S, -sum_w, sum_w)              // |sum_w|
    val underConf    = if (useAdaptive) (absSum < theta(w)) else false.B
    // val trainBit = if (useSelectiveBit) discriminative(w) else true.B
    val trainBit = if (useSelectiveBit) (discriminative(w) || !anyDiscrim) else true.B
    tr_we(w) := ((pred_bit =/= target_bit) || underConf) && trainBit

    if (useAdaptive) {
      when (tr_fire) {
        when (!correct) {
          when (tc(w) >= (thetaSpeed - 1).S) {
            theta(w) := theta(w) + thetaStep.S
            tc(w)    := 0.S
          } .otherwise { tc(w) := tc(w) + 1.S }
        } .elsewhen (underConf) {
          adapt_train := true.B
          when (tc(w) <= -(thetaSpeed - 1).S) {
            theta(w) := Mux(theta(w) > thetaStep.S, theta(w) - thetaStep.S, 0.S)
            tc(w)    := 0.S
          } .otherwise { tc(w) := tc(w) - 1.S }
        }
      }
    }

    for (t <- 0 until Tbl) {
      val delta = Mux(target_bit.asBool, 1.S(5.W), -1.S(5.W))
      val raw   = Mux(tr_we(w), tr_carried_wt(t)(w) + delta, tr_carried_wt(t)(w))
      tr_updated_wt(t)(w) := Mux(raw > 15.S, 15.S, Mux(raw < -16.S, -16.S, raw))
    }
    if (useBias) {
      val b_delta = Mux(target_bit.asBool, 1.S(5.W), -1.S(5.W))
      val b_raw   = Mux(tr_we(w), tr_carried_bia(w) + b_delta, tr_carried_bia(w))
      tr_updated_bias(w) := Mux(b_raw > 15.S, 15.S, Mux(b_raw < -16.S, -16.S, b_raw))
    }
  }

  val tr_any_we = tr_we.asUInt.orR
  // val tr_any_we = false.B  // disable training for now, to save power and avoid interference with ITC testing

  // ── ONE muxed write port per mem (init || training), exactly ONE .write() call site each ──
  for (t <- 0 until Tbl) {
    val init_wr = !wt_init_done && (wt_init_idx < E.U)
    val wr_en   = init_wr || (tr_b_fire && tr_any_we)
    val wr_idx  = Mux(init_wr, wt_init_idx(log2Ceil(E)-1, 0), tr_b_idx(t))
    val wr_data = Mux(init_wr, initRow, tr_updated_wt(t))
    when (wr_en) { weights(t).write(wr_idx, wr_data) }
  }
  if (useBias) {
    val bias_init = !wt_init_done
    val bwr_en    = bias_init || (tr_b_fire && tr_any_we)
    val bwr_idx   = Mux(bias_init, wt_init_idx, tr_b_bias_idx)
    val bwr_data  = Mux(bias_init, initRow, tr_updated_bias)
    when (bwr_en) { biasMem.get.write(bwr_idx, bwr_data) }
  }

  // Convergence observer
  io.tr_event       := tr_b_fire
  io.tr_exact_event := tr_b_fire && (tr_b_fp === tr_actual_fp)

  // Hamming convergence observer (4c-obs)
  val tr_xor = Wire(UInt(F.W)); tr_xor := tr_b_fp ^ tr_actual_fp
  val tr_ham = PopCount(tr_xor)
  io.tr_ham_le2 := tr_b_fire && (tr_ham <= 2.U)
  io.tr_ham_le4 := tr_b_fire && (tr_ham <= 4.U)

  // 4d meta unpack + accuracy/confidence observer (commit-side)
  // Layout: meta(34)=valid, meta(33,30)=min_ham, meta(29,15)=sig, meta(14,0)=fingerprint(4c)
  val carried_snip_valid = RegNext(io.update.bits.meta(34))
  val carried_min_ham    = RegNext(io.update.bits.meta(33, 30))
  val carried_sig        = RegNext(io.update.bits.meta(29, 15))
  val actual_sig         = RegNext(u.target(F - 1, 0))

  io.snip_has_cand_event     := tr_b_fire && carried_snip_valid
  io.snip_pick_match_event   := tr_b_fire && carried_snip_valid && (carried_sig === actual_sig)
  io.snip_min_ham_le2_event  := tr_b_fire && carried_snip_valid && (carried_min_ham <= 2.U)
  io.snip_min_ham_le4_event  := tr_b_fire && carried_snip_valid && (carried_min_ham <= 4.U)
  io.adapt_train_event := adapt_train

  // Full-target attribution for active BLBP overrides. A missing predecessor
  // prediction is classified as predecessor-chain wrong.
  val carried_override    = RegNext(io.update.bits.meta(offOverride))
  val carried_base_valid  = RegNext(io.update.bits.meta(offBaseValid))
  val carried_base_target = RegNext(io.update.bits.meta(offBaseTarget + vaddrBitsExtended - 1, offBaseTarget))
  val carried_new_target  = RegNext(io.update.bits.meta(offNewTarget + vaddrBitsExtended - 1, offNewTarget))
  val actual_target       = tr_b_target
  val attribution_fire    = tr_b_fire && carried_override
  val base_correct        = carried_base_valid && (carried_base_target === actual_target)
  val new_correct         = carried_new_target === actual_target
  val corrected           = attribution_fire && !base_correct && new_correct
  val harmed              = attribution_fire && base_correct && !new_correct
  val still_wrong         = attribution_fire && !base_correct && !new_correct
  val both_correct        = attribution_fire && base_correct && new_correct

  io.indirect_override_event    := attribution_fire
  io.indirect_corrected_event   := corrected
  io.indirect_harmed_event      := harmed
  io.indirect_still_wrong_event := still_wrong

  when (attribution_fire) {
    assert(PopCount(VecInit(Seq(corrected, harmed, still_wrong, both_correct))) === 1.U)
  }

  // Shared set-index hash
  def itcSet(pc: UInt): UInt = fetchIdx(pc)(log2Ceil(itc_nSets) - 1, 0)
  def itcTag(pc: UInt): UInt = fetchIdx(pc)(log2Ceil(itc_nSets) + blbpTagBits - 1, log2Ceil(itc_nSets))

  // ── Commit-side ITC Update RMW ─────────────────────────────────────────────
  val upd_fire = io.update.valid && u.is_commit_update &&
                 u.cfi_is_jalr && !u.cfi_is_ret && u.cfi_taken && u.cfi_idx.valid
  val upd_idx  = itcSet(u.pc)
  val upd_col  = u.cfi_idx.bits

  // ITC commit read DROPPED — carried in meta (TAGE pattern). Unpacked as tr_carried_pool.

  val b_fire   = RegNext(upd_fire, false.B)
  val b_idx    = RegNext(upd_idx)
  val b_col    = RegNext(upd_col)
  val b_target = RegNext(u.target)
  val b_ways   = tr_carried_pool

  val wr_dbg_cnt = RegInit(0.U(8.W))

  // Decompress carried pool for commit-side hit/sb (identity when off)
  val b_tgt = VecInit(b_ways.map(entryTarget))

  val b_hit_oh = VecInit((0 until itc_nWays).map(w =>
    b_ways(w).valid && (if (useTags) (b_ways(w).tag === b_tag) else true.B) &&
    (b_tgt(w) === b_target)))
  val b_hit    = b_hit_oh.asUInt.orR

  io.itc_total_event := b_fire
  io.itc_hit_event   := b_fire && b_hit

  // Region-table allocation (insert only)
  val victim_tgt = if (useRegion) {
    val region_number = b_target(vaddrBitsExtended-1, offsetBits)
    val match_oh = VecInit((0 until nRegions).map(i =>
      region_valid(i) && (region_entries.read(i.U) === region_number)))
    val free_oh  = VecInit(region_valid.map(!_))
    val rand_slot = rand_counter(regionIdxW-1, 0)

    val region_full = !free_oh.asUInt.orR
    val rrip_victim = if (regionRRIP) {
      val rr = region_rrpv.get
      val maxRrpv = rr.reduce((a,b) => Mux(a >= b, a, b))
      PriorityEncoder(rr.map(_ === maxRrpv))
    } else rand_slot

    val alloc_slot = Mux(match_oh.asUInt.orR, PriorityEncoder(match_oh),
                    Mux(free_oh.asUInt.orR,  PriorityEncoder(free_oh), rrip_victim))

    when (b_fire && !b_hit && region_init_done) {
      region_entries.write(alloc_slot, region_number)
      region_valid(alloc_slot)   := true.B
      rand_counter := rand_counter + 17.U
      if (regionRRIP) {
        val rr = region_rrpv.get
        val age = 3.U - rr.reduce((a,b) => Mux(a >= b, a, b))
        when (region_full) { rr.foreach(r => r := r + age) }  // age ONLY when evicting
        rr(alloc_slot) := 2.U                                  // insert value on the actual slot
      }
    }
    if (regionRRIP) {
      when (b_fire && b_hit && match_oh.asUInt.orR) {
        region_rrpv.get(PriorityEncoder(match_oh)) := 0.U      // promote on reuse
      }
    }
    Cat(alloc_slot, b_target(offsetBits-1, 0))
  } else {
    0.U(tgtBits.W)
  }

  // ── ITC Initialization FSM (flat depth, single-entry writes) ─────────────────
  val flatDepth = itc_nSets * bankWidth
  // Tree-pLRU replacement state (W-1 bits per set/col, separate from ITCEntry.rpv)
  val L = log2Ceil(itc_nWays)
  val nNodes = itc_nWays - 1
  val itc_plru_state = if (usePLRU) Some(
    RegInit(VecInit(Seq.fill(flatDepth)(0.U(nNodes.W))))
  ) else None
  val init_done = RegInit(false.B)
  val init_idx  = RegInit(0.U(log2Ceil(flatDepth).W))
  val dbg_cnt   = RegInit(0.U(8.W))
  val init_zero = {
    val e = Wire(new ITCEntry); e.valid := false.B; e.tag := 0.U; e.tgt := 0.U
    e.rpv := (if (useRRIP) 3.U(2.W) else 0.U(2.W))   // RRIP: distant; NRU: 0
    e
  }
  val init_wr = !init_done
  when (!init_done) {
    init_idx := init_idx + 1.U
    when (init_idx === (flatDepth - 1).U) { init_done := true.B }
  }

  if (usePLRU) {
    // ── Tree-pLRU (invalid-first, tree-walk victim, MRU-update on access) ──
    // Shared recurrence: children of node n are 2n+1 (left), 2n+2 (right).
    // Tree bit = 0 → LRU in left subtree, = 1 → LRU in right subtree.
    val inval_oh  = VecInit(b_ways.map(w => !w.valid))
    val has_inval = inval_oh.asUInt.orR
    val tree_vec  = VecInit(itc_plru_state.get(itcAddr(b_idx, b_col)).asBools)

    // ── Victim: descend following LRU pointers, building way MSB-first ──
    val v_node = Wire(Vec(L + 1, UInt(log2Ceil(nNodes + 1).W)))
    val v_way  = Wire(Vec(L + 1, UInt(L.W)))
    v_node(0) := 0.U
    v_way(0)  := 0.U
    for (l <- 0 until L) {
      val bit = tree_vec(v_node(l))
      v_way(l + 1)  := (v_way(l) << 1) | bit
      v_node(l + 1) := (v_node(l) << 1) + 1.U + bit
    }
    val plru_victim = Mux(has_inval, PriorityEncoder(inval_oh), v_way(L))

    // ── MRU update: descend to acc_way, flip path bits to point AWAY ──
    // Uses the IDENTICAL recurrence as the victim walk for node numbering.
    val acc_way = Mux(b_hit, PriorityEncoder(b_hit_oh), plru_victim)
    val u_node  = Wire(Vec(L + 1, UInt(log2Ceil(nNodes + 1).W)))
    u_node(0) := 0.U
    for (l <- 0 until L) {
      val bit = acc_way(L - 1 - l)  // MSB first
      u_node(l + 1) := (u_node(l) << 1) + 1.U + bit
    }

    val plru_next = Wire(Vec(nNodes, Bool()))
    for (i <- 0 until nNodes) {
      val onPath  = VecInit((0 until L).map(l => u_node(l) === i.U)).asUInt.orR
      val levelOf = PriorityEncoder(VecInit((0 until L).map(l => u_node(l) === i.U)))
      plru_next(i) := Mux(onPath, !acc_way((L - 1).U - levelOf), tree_vec(i))
    }

    // Tree state write-back (single call site, combinational read → reg write)
    when (b_fire) {
      itc_plru_state.get(itcAddr(b_idx, b_col)) := plru_next.asUInt
    }

    when (b_fire && wr_dbg_cnt < 20.U) {
      wr_dbg_cnt := wr_dbg_cnt + 1.U
      printf(p"[wr] set=${b_idx} col=${b_col} victim=${plru_victim} hit=${b_hit} tgtfp=0x${Hexadecimal(extractFp(b_target))}\n")
    }

    for (w <- 0 until itc_nWays) {
      val is_victim = !b_hit && (w.U === plru_victim)
      val is_hitway = b_hit && b_hit_oh(w)
      val commit_we = b_fire && (is_victim || is_hitway)

      val e = Wire(new ITCEntry)
      e.valid  := true.B
      e.tag    := (if (useTags) Mux(is_victim, b_tag, b_ways(w).tag) else 0.U)
      e.tgt    := (if (useRegion) Mux(is_victim, victim_tgt, b_ways(w).tgt)
                    else           Mux(is_victim, b_target,   b_ways(w).tgt))
      e.rpv    := 0.U
      // Folded write: single call site, Muxed addr/data, no mask → 1R1W → BRAM
      val we    = init_wr || commit_we
      val waddr = Mux(init_wr, init_idx, itcAddr(b_idx, b_col))
      val wdata = Mux(init_wr, init_zero, e)
      when (we) { itc(w).write(waddr, wdata) }
    }
  } else if (!useRRIP) {
    // ── NRU (reproduces Stage 2 exactly, rpv in {0,1}) ──
    val inval_oh = VecInit(b_ways.map(w => !w.valid))
    val nru_oh   = VecInit(b_ways.map(w => w.rpv === 0.U))
    val victim   = Mux(inval_oh.asUInt.orR, PriorityEncoder(inval_oh),
                   Mux(nru_oh.asUInt.orR,   PriorityEncoder(nru_oh), 0.U))
    val all_used = !inval_oh.asUInt.orR && !nru_oh.asUInt.orR

    when (b_fire && wr_dbg_cnt < 20.U) {
      wr_dbg_cnt := wr_dbg_cnt + 1.U
      printf(p"[wr] set=${b_idx} col=${b_col} victim=${victim} hit=${b_hit} tgtfp=0x${Hexadecimal(extractFp(b_target))}\n")
    }

    for (w <- 0 until itc_nWays) {
      val is_victim = !b_hit && (w.U === victim)
      val is_hitway = b_hit && b_hit_oh(w)
      val clear_nru = all_used && !is_victim
      val commit_we = b_fire && (is_victim || is_hitway || clear_nru)

      val e = Wire(new ITCEntry)
      e.valid  := true.B
      e.tag    := (if (useTags) Mux(is_victim, b_tag, b_ways(w).tag) else 0.U)
      e.tgt    := (if (useRegion) Mux(is_victim, victim_tgt, b_ways(w).tgt)
                    else           Mux(is_victim, b_target,   b_ways(w).tgt))
      e.rpv    := Mux(clear_nru, 0.U, 1.U)
      // Folded write: single call site, Muxed addr/data, no mask → 1R1W → BRAM
      val we    = init_wr || commit_we
      val waddr = Mux(init_wr, init_idx, itcAddr(b_idx, b_col))
      val wdata = Mux(init_wr, init_zero, e)
      when (we) { itc(w).write(waddr, wdata) }
    }
  } else {
    // ── SRRIP (2-bit RRPV; invalid-first, else max-RRPV victim with one-shot aging) ──
    val inval_oh  = VecInit(b_ways.map(w => !w.valid))
    val has_inval = inval_oh.asUInt.orR
    val maxRRPV   = b_ways.map(_.rpv).reduce((a, b) => Mux(a >= b, a, b))
    val delta     = 3.U - maxRRPV                 // aging amount (sum stays <= 3)
    val max_oh    = VecInit(b_ways.map(w => w.rpv === maxRRPV))
    val victim    = Mux(has_inval, PriorityEncoder(inval_oh), PriorityEncoder(max_oh))
    val age_all   = !b_hit && !has_inval          // age the set only on a full miss

    when (b_fire && wr_dbg_cnt < 20.U) {
      wr_dbg_cnt := wr_dbg_cnt + 1.U
      printf(p"[wr] set=${b_idx} col=${b_col} victim=${victim} hit=${b_hit} tgtfp=0x${Hexadecimal(extractFp(b_target))}\n")
    }

    for (w <- 0 until itc_nWays) {
      val is_victim = !b_hit && (w.U === victim)
      val is_hitway = b_hit && b_hit_oh(w)
      val do_age    = age_all && !is_victim
      val commit_we = b_fire && (is_victim || is_hitway || do_age)

      val e = Wire(new ITCEntry)
      e.valid  := true.B
      e.tag    := (if (useTags) Mux(is_victim, b_tag, b_ways(w).tag) else 0.U)
      e.tgt    := (if (useRegion) Mux(is_victim, victim_tgt, b_ways(w).tgt)
                    else           Mux(is_victim, b_target,   b_ways(w).tgt))
      e.rpv    := Mux(is_hitway, 0.U,                       // hit-promotion
                  Mux(is_victim, 2.U,                       // SRRIP long insertion
                                 b_ways(w).rpv + delta))    // aging (do_age)
      // Folded write: single call site, Muxed addr/data, no mask → 1R1W → BRAM
      val we    = init_wr || commit_we
      val waddr = Mux(init_wr, init_idx, itcAddr(b_idx, b_col))
      val wdata = Mux(init_wr, init_zero, e)
      when (we) { itc(w).write(waddr, wdata) }
    }
  }

  // ── Simulation-only evict-then-miss counter ─────────────────────────────
  if (IN_SIMULATION) {
    val inval_oh = VecInit(b_ways.map(w => !w.valid))
    val has_inval = inval_oh.asUInt.orR
    val dbg_victim = if (usePLRU) {
      val dbg_tree = VecInit(itc_plru_state.get(itcAddr(b_idx, b_col)).asBools)
      val dbg_node = Wire(Vec(L + 1, UInt(log2Ceil(nNodes + 1).W)))
      val dbg_way  = Wire(Vec(L + 1, UInt(L.W)))
      dbg_node(0) := 0.U
      dbg_way(0)  := 0.U
      for (l <- 0 until L) {
        val bit = dbg_tree(dbg_node(l))
        dbg_way(l + 1)  := (dbg_way(l) << 1) | bit
        dbg_node(l + 1) := (dbg_node(l) << 1) + 1.U + bit
      }
      Mux(has_inval, PriorityEncoder(inval_oh), dbg_way(L))
    } else if (!useRRIP) {
      val nru_oh = VecInit(b_ways.map(w => w.rpv === 0.U))
      Mux(has_inval, PriorityEncoder(inval_oh),
      Mux(nru_oh.asUInt.orR, PriorityEncoder(nru_oh), 0.U))
    } else {
      val maxRRPV = b_ways.map(_.rpv).reduce((a, b) => Mux(a >= b, a, b))
      val max_oh  = VecInit(b_ways.map(w => w.rpv === maxRRPV))
      Mux(has_inval, PriorityEncoder(inval_oh), PriorityEncoder(max_oh))
    }

    val shadow_set   = RegInit(0.U(log2Ceil(itc_nSets).W))
    val shadow_col   = RegInit(0.U(log2Ceil(bankWidth).W))
    val shadow_tgt   = RegInit(0.U(tgtBits.W))
    val shadow_valid = RegInit(false.B)
    val evict_miss_cnt = RegInit(0.U(32.W))

    when (b_fire && !b_hit) {
      shadow_set   := b_idx
      shadow_col   := b_col
      shadow_tgt   := b_tgt(dbg_victim)
      shadow_valid := true.B
    }

    val same_sc = (b_idx === shadow_set) && (b_col === shadow_col)
    when (b_fire && shadow_valid && same_sc) {
      when (!b_hit && (b_target === shadow_tgt)) {
        evict_miss_cnt := evict_miss_cnt + 1.U
        printf(p"[evict_then_miss] cnt=${evict_miss_cnt + 1.U} set=${b_idx} col=${b_col} tgt=0x${Hexadecimal(b_target)}\n")
      }
      shadow_valid := false.B
    }
  }

  // ── Predict-side ITC Read — folded: single-column pool directly ────────────
  // Derive snip_col at s2 from f2 response (is_jal stable s1→s3; taken stable
  // for non-BR entries). Read address = Cat(set, s2_snip_col) → single column.
  val s2_resp = io.resp_in(0).f2
  val s2_jalr_mask = VecInit((0 until bankWidth).map(w =>
    s2_resp(w).is_jal && s2_resp(w).taken))
  val s2_snip_col   = PriorityEncoder(s2_jalr_mask)
  val s2_itc_set    = RegNext(RegNext(itcSet(io.f0_pc)))  // double RegNext → s2 stage
  val s2_itc_tag    = RegNext(RegNext(itcTag(io.f0_pc)))
  val s2_read_addr  = itcAddr(s2_itc_set, s2_snip_col)
  val s3_pool = VecInit((0 until itc_nWays).map(w =>
    itc(w).read(s2_read_addr, s2_valid)))

  // Carry s2_snip_col to s3 — one source, used for both read and override
  val snip_col     = RegNext(s2_snip_col)
  val s3_has_jalr  = RegNext(s2_jalr_mask.asUInt.orR)
  val s3_itc_tag   = RegNext(s2_itc_tag)
  val s3_itc_set   = RegNext(s2_itc_set)

  // Decompress pool (identity when useRegion=false)
  val s3_pool_tgt = VecInit(s3_pool.map(entryTarget))

  val s3_resp = io.resp_in(0).f3
  val s3_taken_mask = VecInit((0 until bankWidth).map(w => s3_resp(w).taken))
  val s3_has_taken = s3_taken_mask.asUInt.orR

  // ── Vintage-match diagnostic: compare s3 meta idh vs commit carried_hist ──
  if (IN_SIMULATION && usePrivateHist) {
    when (tr_fire) {
      val carried_idx = tblIdxMix(u.pc, io.update.bits.ghist, carried_hist, 1)
      printf(p"[vintage] pc=0x${Hexadecimal(fetchIdx(u.pc))} " +
        p"carried_idh=0x${Hexadecimal(carried_hist)} live_idh=0x${Hexadecimal(blbp_ghist)} " +
        p"train_idx=${carried_idx} fp_match=${io.tr_exact_event}\n")
      printf(p"[carry] meta_hist=0x${Hexadecimal(io.update.bits.meta(offHist + W_HIST - 1, offHist))} " +
        p"carried=0x${Hexadecimal(carried_hist)} " +
        p"meta_lo=0x${Hexadecimal(io.update.bits.meta(41,0))}\n")
      printf(p"[carry2] meta_hist=0x${Hexadecimal(io.update.bits.meta(offHist+W_HIST-1, offHist))} " +
        p"carried_hist=0x${Hexadecimal(carried_hist)} " +
        p"meta_fp=0x${Hexadecimal(io.update.bits.meta(F-1,0))} " +
        p"tr_b_fp=0x${Hexadecimal(tr_b_fp)}\n")
    }
    when (s3_valid && s3_has_jalr) {
      printf(p"[pack] blbp_ghist=0x${Hexadecimal(blbp_ghist)} s3_blbp_ghist=0x${Hexadecimal(s3_blbp_ghist)}\n")
    }
  }

  // Fingerprint observer
  io.fp_computed_event := s3_valid && s3_has_taken
  io.fp_nonzero_event  := s3_valid && s3_has_taken && (s3_fingerprint =/= fp_untrained)

  // ── Hamming-match + min-select (s3) ────────────────────────────────────────
  // JALR proxy: is_jal && taken. Already computed at s2, carried via RegNext.
  val cand_fp    = VecInit(s3_pool_tgt.map(t => extractFp(t)))
  val cand_valid = VecInit(s3_pool.map { e =>
    e.valid && (if (useTags) (e.tag === s3_itc_tag) else true.B)
  })

  // Predict-side which-bits mask: only discriminative bits count toward the gate.
  // Mirrors training-side suppression so the confidence gate ignores untrained bits.
  def discrimP(w: Int): Bool = {
    val anyOne  = (0 until itc_nWays).map(c => cand_valid(c) &&  cand_fp(c)(w)).reduce(_ || _)
    val anyZero = (0 until itc_nWays).map(c => cand_valid(c) && !cand_fp(c)(w)).reduce(_ || _)
    anyOne && anyZero
  }
  val discrimMask =
    if (useSelectiveBit) VecInit((0 until F).map(w => discrimP(w))).asUInt
    else                 ((BigInt(1) << F) - 1).U(F.W)

  val ham = VecInit((0 until itc_nWays).map { w =>
    PopCount(((s3_fingerprint ^ cand_fp(w)) & discrimMask).asUInt)  // masked
  })
  // Key: top bit = !valid (invalid sorts above all valid), lower bits = ham
  val key = VecInit((0 until itc_nWays).map { w =>
    Cat(!cand_valid(w), ham(w))  // 5-bit key
  })

  // Balanced-tree reduce: same value as linear .reduce (addition is associative),
  // but ~log2(n) logic depth instead of n. Preserves left-to-right order so the
  // lower-index element is always the "a" side in pairwise combine (tie-break).
  def treeReduce[T](xs: Seq[T])(f: (T, T) => T): T = {
    require(xs.nonEmpty)
    if (xs.length == 1) xs.head
    else {
      val paired = xs.grouped(2).map {
        case Seq(a, b) => f(a, b)
        case Seq(a)    => a            // odd tail passes through at its position
      }.toSeq
      treeReduce(paired)(f)
    }
  }

  // Pairwise min-select over itc_nWays ways, tie-break lower way
  def reduceMin(pairs: Seq[(UInt, UInt)]): (UInt, UInt) = {
    if (pairs.length == 1) {
      pairs.head
    } else {
      val half = pairs.grouped(2).map { g =>
        val (a_idx, a_key) = g(0)
        val (b_idx, b_key) = if (g.length == 2) g(1) else (a_idx, a_key) // pad odd
        (Mux(b_key <= a_key, b_idx, a_idx), Mux(b_key <= a_key, b_key, a_key))
      }.toSeq
      reduceMin(half)
    }
  }

  val snip_sel = if (useDotProduct) {
    // ── dotw: 15-term signed sum, tree instead of linear chain ──
    // +& never truncates, addition is associative => value identical to .reduce(_ +& _)
    val dotw = VecInit((0 until itc_nWays).map { w =>
      treeReduce((0 until F).map { k =>
        val contrib = Mux(cand_fp(w)(k).asBool, s3_sum(k), -s3_sum(k))
        Mux(discrimMask(k), contrib, 0.S)   // only discriminative bits vote
      })(_ +& _)
    })

    // ── argmax over VALID ways, tie -> lower index ──
    // Pairwise combine reproduces the old foldLeft exactly:
    //   b wins  iff  b.valid && (!a.valid || b.dot > a.dot)     (strict > => tie keeps a = lower idx)
    // 'a' is always the lower-index side because treeReduce preserves order.
    def selBetter(a: (UInt, Bool, SInt), b: (UInt, Bool, SInt)): (UInt, Bool, SInt) = {
      val (aIdx, aVal, aDot) = a
      val (bIdx, bVal, bDot) = b
      val bWins = bVal && (!aVal || (bDot > aDot))
      (Mux(bWins, bIdx, aIdx), aVal || bVal, Mux(bWins, bDot, aDot))
    }
    val cands = (0 until itc_nWays).map(w =>
      (w.U(log2Ceil(itc_nWays).W), cand_valid(w), dotw(w)))
    val (sel, _, _) = treeReduce(cands)(selBetter)

    if (IN_SIMULATION) {
      val dbg_en = s3_valid && s3_has_jalr && cand_valid.asUInt.orR
      val dbg_cnt = RegInit(0.U(8.W))
      when (dbg_en && dbg_cnt < 20.U) {
        dbg_cnt := dbg_cnt + 1.U
        printf(p"[dot] sel=$sel fp=0x${Hexadecimal(s3_fingerprint)} mask=0x${Hexadecimal(discrimMask)}\n")
        printf(p"[rd] set=${s3_itc_set} col=${snip_col}\n")
        for (w <- 0 until itc_nWays) {
          when (cand_valid(w)) {
            printf(p"   way$w cand_fp=0x${Hexadecimal(cand_fp(w))} dot=${dotw(w)}\n")
          }
        }
      }
    }
    sel
  } else {
    val (sel, _) = reduceMin((0 until itc_nWays).map(w =>
      (w.U(log2Ceil(itc_nWays).W), key(w))))
    sel
  }

  val snip_valid   = cand_valid.asUInt.orR && s3_has_jalr
  val snip_target  = s3_pool_tgt(snip_sel)
  val snip_sig     = snip_target(F - 1, 0)
  val snip_min_ham = ham(snip_sel)
  val base_prediction = s3_resp(snip_col).predicted_pc
  val snip_override = snip_valid && (snip_min_ham <= override_thresh.U)

  // Carry predict bias + weights + ITC pool + snip fields through f3_meta (TAGE pattern)
  // Layout: attribution | lhist | hist | pool | wt | bias | valid|min_ham|sig|fingerprint
  io.f3_meta := Cat(snip_target, base_prediction.bits, base_prediction.valid, snip_override,
    s3_local_hist, s3_blbp_ghist, s3_pool.asUInt, s3_wt.asUInt,
    (if (useBias) s3_bia.asUInt else 0.U(0.W)),
    snip_valid, snip_min_ham, snip_sig, s3_fingerprint)

  // F3 target override: correct predicted_pc when confident (first active change)
  when (snip_override) {
    io.resp.f3(snip_col).predicted_pc.valid := true.B
    io.resp.f3(snip_col).predicted_pc.bits  := snip_target
  }

  // All-column observers dropped (folded read provides single column only)
  io.pred_taken_event          := s3_valid && s3_has_taken
  io.pred_pool_nonempty_event  := false.B
  io.pred_target_in_pool_event := false.B
  io.pred_pool_saturated_event := false.B
}
