package boom.v3.ifu

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import boom.v3.common._

class SNIPBranchPredictorBank(implicit p: Parameters) extends BranchPredictorBank()(p) {

  def itc_nSets = p(BoomSnipITCSets)
  def itc_nWays = p(BoomSnipITCWays)
  val useSnipAdaptCoeff = p(BoomSnipAdaptCoeff)
  val coeffShift = p(BoomSnipCoeffShift)
  val useSnipAdaptTheta = p(BoomSnipAdaptTheta)
  val thetaInit = p(BoomSnipThetaInit)

  class ITCEntry extends Bundle {
    val valid  = Bool()
    val target = UInt(vaddrBitsExtended.W)
  }

  // PC-addressed index — full-entry write (no mask) → BRAM-compatible.
  // The instruction COLUMN is deliberately NOT part of the address: a fetch packet
  // holds at most one executed JALR, so the column is redundant with the PC, and
  // sourcing it from the L2 prediction made the read row depend on the L2 being
  // right. Widening the set index by log2(bankWidth) keeps the entry count (and
  // the mems declaration) identical.
  val itcIdxW = log2Ceil(itc_nSets * bankWidth)
  val flatDepth = itc_nSets * bankWidth
  val itc = Seq.fill(itc_nWays) { SyncReadMem(flatDepth, new ITCEntry) }
  def itcAddr(idx: UInt): UInt = idx(itcIdxW - 1, 0)

  // ── Tree-pLRU replacement state ─────────────────────────────────────────────
  // One tree per indexed row (flatDepth rows), itc_nWays - 1 bits per tree.
  // A node's bit names the side that holds the LRU half (0 → left, 1 → right),
  // so descending from the root lands on the least-recently-used way, and an
  // access flips every node on its path to point away from it.
  //
  // The tree is built by recursive halving, so ANY itc_nWays >= 2 works — not
  // just powers of two — while collapsing to the usual heap-numbered tree when
  // itc_nWays is a power of two. Leaves self-loop (both children = self), so
  // the unrolled descent always terminates on a leaf even though an early leaf
  // is less deep than the tree's nominal height.
  require(itc_nWays >= 2, s"ITC tree-pLRU needs at least 2 ways, got $itc_nWays")
  val plruL     = log2Ceil(itc_nWays)   // tree height = levels of decisions
  val plruNodes = itc_nWays - 1         // internal nodes = state bits per row
  val plruWayW  = math.max(1, log2Ceil(itc_nWays))      // way indices, 0..nWays-1
  val plruCmpW  = math.max(1, log2Ceil(itc_nWays + 1))  // node bounds, 0..nWays
  val plruBitW  = math.max(1, log2Ceil(plruNodes))

  import scala.collection.mutable.ArrayBuffer
  val pLo  = ArrayBuffer[Int]()   // node → way range [lo, hi) it covers
  val pHi  = ArrayBuffer[Int]()
  val pMid = ArrayBuffer[Int]()   // internal node → split point
  val pBit = ArrayBuffer[Int]()   // internal node → state bit (leaves: unused)
  val pL   = ArrayBuffer[Int]()   // node → children (leaves: self-loop)
  val pR   = ArrayBuffer[Int]()
  val pWay = ArrayBuffer[Int]()   // leaf → way index (-1 for internal nodes)

  def buildPlru(lo: Int, hi: Int): Int = {
    val me = pLo.length
    pLo += lo; pHi += hi; pMid += -1; pBit += 0
    pL += me; pR += me; pWay += -1
    if (hi - lo == 1) {
      pWay(me) = lo
    } else {
      val mid = (lo + hi) / 2
      pMid(me) = mid
      pL(me) = buildPlru(lo, mid)
      pR(me) = buildPlru(mid, hi)
    }
    me
  }
  val plruRoot = buildPlru(0, itc_nWays)
  // Bit indices follow node (pre-order) order; internal nodes only.
  var plruBitCursor = 0
  for (i <- pLo.indices) {
    if (pWay(i) < 0) { pBit(i) = plruBitCursor; plruBitCursor += 1 }
  }
  require(plruBitCursor == plruNodes)
  // Internal nodes in bit-index order — the reverse map the update walk needs.
  val plruBitNode = pLo.indices.filter(i => pWay(i) < 0)

  val plruNodeIdxW  = log2Ceil(pLo.length)
  val plruBitIdxVec = VecInit(pLo.indices.map(i => pBit(i).U(plruBitW.W)))
  val plruLeftVec   = VecInit(pLo.indices.map(i => pL(i).U(plruNodeIdxW.W)))
  val plruRightVec  = VecInit(pLo.indices.map(i => pR(i).U(plruNodeIdxW.W)))
  val plruWayVec    = VecInit(pLo.indices.map(i => math.max(0, pWay(i)).U(plruWayW.W)))

  // ── Fingerprint datapath constants ──────────────────────────────────────────
  val F = 15
  val Tbl = 8
  val E = 512
  val nbias = 1024

  val histLens = Seq(0, 2, 4, 8, 12, 18, 28, 42).map(_ min globalHistoryLength)

  // ── Path-history windows (reference SNIP indexing) ──────────────────────────
  // Jimenez's SNIP builds the weight-table index from the PC and the *addresses*
  // of recent branches, and feeds the direction history into the dot product as a
  // ±1 feature. GlobalHistory.path_history is exactly that ring (snipRingEntries
  // entries of snipPcBits branch-PC bits, newest entry at the low end), so with
  // useSnipPathRing the index folds the ring and the direction history becomes
  // the per-learner ±1. With the flag off the index keeps folding the direction
  // history, as before.
  val pathLens = Seq(0, 2, 4, 8, 12, 18, 28, 42)
  // Branch depth each window reaches — the direction bit that learner pairs with,
  // mirroring the reference's "learner i sees direction bit i".
  val pathDepths = pathLens.map(l => if (l == 0) 0 else (l + snipPcBits - 1) / snipPcBits)
  if (useSnipPathRing) {
    // GlobalHistory.update only advances the ring in its single-bank branch; with
    // two banks path_history is never written and would index as X.
    require(nBanks == 1,
      s"useSnipPathRing needs nBanks == 1 (got $nBanks): the two-bank history path " +
      s"does not maintain path_history")
    require(pathLens.max <= snipRingEntries * snipPcBits,
      s"pathLens.max (${pathLens.max}) exceeds the path ring (${snipRingEntries * snipPcBits} bits)")
    require(pathDepths.max <= globalHistoryLength,
      s"path depth ${pathDepths.max} exceeds globalHistoryLength ($globalHistoryLength)")
  }

  // Reference SNIP starts its coefficients at max(1/(0.059+0.006*d), 4.3) and
  // adapts them online. The port keeps that schedule sampled at its own window
  // depths d = histLens(t) = 0/2/4/8/12/18/28/42, scaled x4 and rounded to
  // 68/56/48/37/31/24/17/17 (the two trailing 17s both sit on the 4.3 floor).
  // With useSnipAdaptCoeff the coefficients become registers, read at predict and
  // updated in place at commit like the tree-pLRU state — no meta snapshot,
  // because the commit side never reads them (the error signal comes from the
  // carried fingerprint).
  // Unlike the pLRU state their *updates* do consume carried predict-time state,
  // which is why the votes below are taken from the carried weights.
  val coeffs_init = Seq(68, 56, 48, 37, 31, 24, 17, 17)
  val coeffBias_init = 68
  // Constant path (useSnipAdaptCoeff = false), exactly as before.
  val coeffs    = VecInit(coeffs_init.map(_.S(8.W)))
  val coeffBias = coeffBias_init.S(8.W)
  // Adaptive path: the coefficients live in Q(CQ_W-CQ_FRAC).CQ_FRAC and move by
  // `coeff_q >> coeffShift` per vote, i.e. a *relative* step of 2^-coeffShift —
  // the fixed-point analogue of the reference's `coeff *= factor` / `/= factor`
  // (factor = 1.00000455 -> 2^-17.8, so the default shift of 18 matches it).
  // The step truncates to zero below coeff_q = 2^coeffShift, i.e. below an
  // effective coefficient of 2^(coeffShift - CQ_FRAC) = 4.0 at the defaults; that
  // is why CQ_FRAC is 16 and not tighter — at CQ_FRAC = 12 the floor would sit at
  // 64 and every coefficient but the first would freeze immediately. At the
  // starting values every product has CQ_FRAC zero low bits, so
  // (w * coeff_q) >> CQ_FRAC stays bit-identical to the integer path below.
  val CQ_FRAC = 16
  val CQ_W    = 25                 // sign + 24 bits; ceiling 255 << CQ_FRAC fits
  val CQ_MIN  = 1 << CQ_FRAC
  val CQ_MAX  = 255 << CQ_FRAC
  require(coeffShift < CQ_W, s"coeffShift ($coeffShift) must be below CQ_W ($CQ_W)")
  require((coeffs_init.min << CQ_FRAC) >= (1 << coeffShift),
    s"coeffShift ($coeffShift) would freeze every coefficient at or below " +
    s"${1 << (coeffShift - CQ_FRAC)}; the smallest starting value is ${coeffs_init.min}")
  val coeffs_q    = if (useSnipAdaptCoeff) Some(
    RegInit(VecInit(coeffs_init.map(c => (c << CQ_FRAC).S(CQ_W.W))))) else None
  val coeffBias_q = if (useSnipAdaptCoeff) Some(
    RegInit((coeffBias_init << CQ_FRAC).S(CQ_W.W))) else None

  // Adaptive training threshold (reference predictor.cc:296-315). The reference's
  // |yout| reaches ~16*Σcoeff ≈ 4768, so its initial 240 lands in the same range
  // as this port's |sum_w| and carries over unchanged. Its `tc` counter is reset
  // on every step it takes, so it never accumulates: theta simply walks by
  // (mispredicted bits - correct-but-under-confident bits) per training event,
  // clamped to the reference's 0..4095.
  val THETA_W = 13                    // 0..4095 with a sign bit
  val theta = if (useSnipAdaptTheta) Some(RegInit(thetaInit.S(THETA_W.W))) else None
  require(!useSnipAdaptTheta || (thetaInit >= 0 && thetaInit <= 4095),
    s"thetaInit ($thetaInit) is outside the reference's 0..4095 range")

  val weights = Seq.fill(Tbl)(SyncReadMem(E, Vec(F, SInt(5.W))))
  val biasMem = SyncReadMem(nbias, Vec(F, SInt(5.W)))

  val fp_untrained = ((1 << F) - 1).U(F.W)

  // ── Meta layout constants (LSB-first) ─────────────────────────────────────
  // [W_FP-1:0] fp (15) | [W_SIG+W_FP-1:W_FP] sig (15) |
  // [W_HAM+W_SIG+W_FP-1:W_SIG+W_FP] min_ham (4) |
  // [W_BASE-1:W_HAM+W_SIG+W_FP] valid (1) → W_BASE = 35
  val W_FP = F; val W_SIG = F; val W_HAM = 4; val W_VALID = 1
  val W_BASE = W_FP + W_SIG + W_HAM + W_VALID  // 35
  val W_BIAS  = F * 5                              // 75
  val W_WT    = Tbl * F * 5                        // 600
  val W_ITC_ENTRY = 1 + vaddrBitsExtended   // valid(1) + target (replacement lives in the tree)
  val W_POOL  = itc_nWays * W_ITC_ENTRY
  val offBias = W_BASE                             // 35
  val offWt   = offBias + W_BIAS                   // 110
  val offPool = offWt   + W_WT                     // 710
  // Attribution fields are appended above the existing layout so the low-bit
  // fingerprint/signature offsets remain stable.
  // Step-2: the override is unconditional, so the baseline — what the frontend
  // would have consumed without us — is the earlier banks' prediction at the
  // committed CFI's column. That column is known only at commit, so carry the
  // pre-override prediction for every column.
  // The override enable is not carried separately: with the Hamming gate gone it
  // is bit-for-bit snip_valid, which already sits in the base field below.
  val offBaseOH    = offPool + W_POOL
  val offBaseTgt   = offBaseOH + bankWidth
  val offNewTarget = offBaseTgt + bankWidth * vaddrBitsExtended
  // Path-ring slice the weight-table index needs, carried so the commit side can
  // recompute the very same rows: the bank update port only carries the direction
  // history, and the ring is not part of it.
  val W_PATH  = if (useSnipPathRing) pathLens.max else 0
  val offPath = offNewTarget + vaddrBitsExtended
  // Per-bit "was this bit under-confident?" flags for the adaptive-threshold
  // gate. They are evaluated at prediction time against the live theta, because
  // the commit side no longer computes the sums that |y| comes from; theta moves
  // by at most F per training event, so the vintage difference is negligible.
  val W_UNDER  = if (useSnipAdaptTheta) F else 0
  val offUnder = offPath + W_PATH

  override val metaSz = offUnder + W_UNDER
  require(metaSz <= bpdMaxMetaLength,
    s"SNIP metaSz ($metaSz) exceeds bpdMaxMetaLength ($bpdMaxMetaLength) " +
    s"— reduce itc_nWays ($itc_nWays) or increase bpdMaxMetaLength")

  override val mems =
    Seq.tabulate(itc_nWays)(w => (s"snip_itc_way$w", flatDepth, 1 + vaddrBitsExtended)) ++
    Seq(("snip_itc_plru", flatDepth, plruNodes)) ++
    Seq.tabulate(Tbl)(t => (s"snip_weight$t", E, F * 5)) :+
    ("snip_bias", nbias, F * 5)

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
  def tblIdx(pc: UInt, ghist: UInt, path: UInt, t: Int): UInt = {
    val fold = if (useSnipPathRing) foldHist(path, pathLens(t)) else foldHist(ghist, histLens(t))
    (fetchIdx(pc) ^ fold)(log2Ceil(E) - 1, 0)
  }

  // Target fingerprint extraction: 0xb5ffa skip bits → compacted 15-bit value
  def extractFp(target: UInt): UInt = {
    val bits = Seq(1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 16, 17, 19)
    VecInit(bits.map(b => target(b))).asUInt
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
  //   s2_sum(w) = Σ_t (±1_t * s2_wt(t)(w) * coeffs(t)) + (s2_bia(w) * coeffBias)
  //   s3_sum = RegNext(s2_sum); fp_bit = (s3_sum(w) >= 0.S)
  // ±1_t is the depth-aligned direction bit; it is always +1 when path mode is off.
  // The commit side never recomputes this sum: the fingerprint it produces is
  // carried in f3_meta. That is cheaper, and since the coefficients below move
  // with the commit stream a recomputation could no longer reproduce the
  // prediction anyway — the carried fingerprint IS what was predicted.
  val biasIdx = fetchIdx(s1_pc)(log2Ceil(nbias) - 1, 0)
  val readEn  = s1_valid && wt_init_done

  val pred_path = if (useSnipPathRing) io.f1_path else 0.U
  val s2_wt  = VecInit((0 until Tbl).map { t => weights(t).read(tblIdx(s1_pc, io.f1_ghist, pred_path, t), readEn) })
  val s2_bia = biasMem.read(biasIdx, readEn)

  // Per-learner ±1, registered so it pairs with the s2 weight read of the same
  // packet. Negation happens after the coefficient product: -(-16) does not fit
  // in SInt(5.W), the product is wide enough.
  val s2_sign = if (useSnipPathRing) Some(RegNext(VecInit((0 until Tbl).map { t =>
    if (pathDepths(t) == 0) true.B else io.f1_ghist(pathDepths(t) - 1)
  }))) else None

  val s2_sum = VecInit((0 until F).map { w =>
    // `+&` (width-growing), not `+`: Chisel's `+` wraps at the wider operand's
    // width, so a wrapping adder would silently truncate the fingerprint sum —
    // the constant vector's worst case alone is 16*298 = 4768, past the SInt(13)
    // that a 5x8 product chain would otherwise land in.
    if (useSnipAdaptCoeff) {
      // Q path: multiply by the full-precision coefficient and shift once after
      // the sum, so the fraction is not lost term by term.
      val qterms = (0 until Tbl).map { t =>
        val p = s2_wt(t)(w) * coeffs_q.get(t)
        if (useSnipPathRing) Mux(s2_sign.get(t), p, -p) else p
      }
      (qterms.reduce(_ +& _) +& (s2_bia(w) * coeffBias_q.get)) >> CQ_FRAC
    } else {
      val terms = (0 until Tbl).map { t =>
        val p = (s2_wt(t)(w) * coeffs(t)).asSInt
        if (useSnipPathRing) Mux(s2_sign.get(t), p, -p) else p
      }
      (terms.reduce(_ +& _) +& (s2_bia(w) * coeffBias).asSInt).asSInt
    }
  })

  val s3_sum = RegNext(s2_sum)
  val s3_bia = RegNext(s2_bia)
  val s3_wt  = RegNext(s2_wt)
  val s3_fingerprint = VecInit(s3_sum.map(s => (s >= 0.S).asUInt)).asUInt
  // |sum_w| < theta, per bit: the reference's under-confident test, evaluated at
  // prediction vintage (see W_UNDER above).
  val s3_under = if (useSnipAdaptTheta) Some(VecInit(s3_sum.map { s =>
    Mux(s >= 0.S, s, 0.S -& s) < theta.get
  })) else None

  // ── Commit-side Training RMW ────────────────────────────────────────────────
  val u        = io.update.bits
  val tr_fire  = io.update.valid && u.is_commit_update &&
                 u.cfi_is_jalr && !u.cfi_is_ret && u.cfi_taken && u.cfi_idx.valid

  // commit-side reads DROPPED — weights + ITC carried in meta (TAGE pattern).
  // The path ring rides the meta too, so the commit side recomputes the same rows.
  val upd_path = if (useSnipPathRing) io.update.bits.meta(offPath + W_PATH - 1, offPath) else 0.U
  def tblIdxUpd(t: Int): UInt = tblIdx(u.pc, io.update.bits.ghist, upd_path, t)

  // 1 cycle later: unpack carried snapshots
  val tr_b_fire      = RegNext(tr_fire, false.B)
  val tr_b_target    = RegNext(u.target)
  val tr_b_fp        = RegNext(io.update.bits.meta(F - 1, 0))
  val tr_carried_bia = RegNext(io.update.bits.meta(offBias + W_BIAS - 1, offBias)).asTypeOf(Vec(F, SInt(5.W)))
  val tr_carried_wt  = RegNext(io.update.bits.meta(offWt   + W_WT   - 1, offWt  )).asTypeOf(Vec(Tbl, Vec(F, SInt(5.W))))
  val tr_carried_pool= RegNext(io.update.bits.meta(offPool + W_POOL - 1, offPool)).asTypeOf(Vec(itc_nWays, new ITCEntry))
  // Same ±1 vintage as the predict side: io.update.bits.ghist is the fetch-time
  // snapshot the prediction was made with, so both sides agree bit-for-bit.
  val tr_b_sign      = if (useSnipPathRing) Some(RegNext(VecInit((0 until Tbl).map { t =>
    if (pathDepths(t) == 0) true.B else io.update.bits.ghist(pathDepths(t) - 1)
  }))) else None
  // Carried per-bit under-confidence flags (see W_UNDER): with the adaptive
  // threshold, a bit is skipped only when it was predicted correctly AND
  // confidently, |y| >= theta (reference predictor.cc:315).
  val tr_b_under     = if (useSnipAdaptTheta) Some(
    RegNext(io.update.bits.meta(offUnder + W_UNDER - 1, offUnder))) else None
  val tr_b_idx       = RegInit(VecInit(Seq.fill(Tbl)(0.U(log2Ceil(E).W))))
  val tr_b_bias_idx  = RegInit(0.U(log2Ceil(nbias).W))
  when (tr_fire) {
    tr_b_idx      := VecInit((0 until Tbl).map(t => tblIdxUpd(t)))
    tr_b_bias_idx := fetchIdx(u.pc)(log2Ceil(nbias) - 1, 0)
  }

  // Training RMW at tr_b_fire: one write per table, all F bits updated in parallel
  val tr_actual_fp = extractFp(tr_b_target)

  // Per-bit train_we and updated rows (combinational, computed at tr_b_fire).
  // tr_we comes straight from the carried fingerprint — that IS what this packet
  // predicted — so the commit side needs neither a dot product nor the
  // coefficients to know which bits went wrong.
  val tr_we    = Wire(Vec(F, Bool()))   // the bit was predicted wrong
  val tr_train = Wire(Vec(F, Bool()))   // ... and this is the bit's train gate
  val tr_updated_wt = Wire(Vec(Tbl, Vec(F, SInt(5.W))))
  val tr_updated_bias = Wire(Vec(F, SInt(5.W)))

  for (w <- 0 until F) {
    val target_bit = tr_actual_fp(w)
    val under_bit  = if (useSnipAdaptTheta) tr_b_under.get(w).asBool else false.B
    tr_we(w)    := (tr_b_fp(w) =/= target_bit)
    tr_train(w) := tr_we(w) || under_bit

    for (t <- 0 until Tbl) {
      val delta = Mux(target_bit.asBool, 1.S(5.W), -1.S(5.W))
      // The ±1 feature multiplies the gradient too: the reference trains
      // `w += (history_bit == target_bit) ? +1 : -1`, i.e. sign_t * sign(target).
      // Its contribution to sum_w is sign_t * w, so a learner currently seeing -1
      // must be driven opposite to the target bit.
      val delta_t = if (useSnipPathRing) Mux(tr_b_sign.get(t), delta, -delta) else delta
      // `+&`, not `+`: Chisel's `+` is a wrapping add, so at SInt(5.W) a weight of
      // +15 stepped by +1 would wrap straight to -16 and the saturation below
      // would be dead code. The reference really does clamp at
      // max_weight/min_weight, so the sum has to be one bit wide enough to see it.
      val raw   = Mux(tr_train(w), tr_carried_wt(t)(w) +& delta_t, tr_carried_wt(t)(w))
      tr_updated_wt(t)(w) := Mux(raw > 15.S, 15.S, Mux(raw < -16.S, -16.S, raw))
    }
    val b_delta = Mux(target_bit.asBool, 1.S(5.W), -1.S(5.W))
    val b_raw   = Mux(tr_train(w), tr_carried_bia(w) +& b_delta, tr_carried_bia(w))
    tr_updated_bias(w) := Mux(b_raw > 15.S, 15.S, Mux(b_raw < -16.S, -16.S, b_raw))
  }

  // ── Adaptive coefficients ──
  // The reference moves each learner's coefficient one multiplicative step per
  // trained bit, according to whether that learner's own vote agreed with the
  // target bit:
  //   int sum = h * weights[idx][i];
  //   if ((sum >= 0) == address_bit) coeff *= factor; else coeff /= factor;
  // The step is `coeff_q >> coeffShift` — a *relative*, scale-free step of
  // 2^-18 by default, against the reference's factor = 1.00000455 = 2^-17.8 —
  // summed over whatever bits this event trains and applied to the live register.
  // (Chisel's `+` wraps at the wider operand's width, hence `+&` throughout: a
  // wrapping accumulator would fold the per-bit votes together and a wrapping
  // register add would send a coefficient at the ceiling down to the floor.)
  // Votes come from the carried weights — the ones that scored this prediction,
  // matching the reference's `h * weights[idx][i]`; the reference reads that
  // weight *after* stepping it, which differs from the port only when the
  // pre-update weight is exactly 0, where the port (like the reference's own bias
  // path) counts a zero weight as a positive vote. The bias has no ±1 feature, so
  // its vote is the weight's own sign, exactly as in the reference. Gated on
  // tr_b_fire — the cycle where the tr_b_* snapshots are the ones this packet
  // trained with — and on the weight-table init FSM having finished, since an
  // early commit would otherwise latch an X read out of the uninitialized mems
  // into the coefficients, which nothing ever re-initializes.
  val adapt_train = if (useSnipAdaptCoeff) {
    val adapt_en = tr_b_fire && wt_init_done
    val dCoeff = (0 until Tbl).map { t =>
      val step = coeffs_q.get(t) >> coeffShift
      (0 until F).map { w =>
        val sign_t = if (useSnipPathRing) tr_b_sign.get(t) else true.B
        // (h * w >= 0), i.e. the learner's contribution to sum_w was non-negative.
        val votePos = Mux(sign_t, tr_carried_wt(t)(w) >= 0.S, tr_carried_wt(t)(w) <= 0.S)
        Mux(tr_train(w), Mux(votePos === tr_actual_fp(w).asBool, step, -step), 0.S)
      }.reduce(_ +& _)
    }
    val dBias = {
      val step = coeffBias_q.get >> coeffShift
      (0 until F).map { w =>
        val votePos = tr_carried_bia(w) >= 0.S
        Mux(tr_train(w), Mux(votePos === tr_actual_fp(w).asBool, step, -step), 0.S)
      }.reduce(_ +& _)
    }

    when (adapt_en) {
      for (t <- 0 until Tbl) {
        val raw = coeffs_q.get(t) +& dCoeff(t)
        coeffs_q.get(t) := Mux(raw > CQ_MAX.S, CQ_MAX.S, Mux(raw < CQ_MIN.S, CQ_MIN.S, raw))
      }
      val braw = coeffBias_q.get +& dBias
      coeffBias_q.get := Mux(braw > CQ_MAX.S, CQ_MAX.S, Mux(braw < CQ_MIN.S, CQ_MIN.S, braw))
    }

    // Single-cycle event: this training event actually moved a coefficient.
    adapt_en && (dCoeff.map(_ =/= 0.S).reduce(_ || _) || dBias =/= 0.S)
  } else false.B
  io.adapt_train_event := adapt_train

  // The write enables follow the train gate, not just the mispredictions: with
  // the adaptive threshold a correct-but-under-confident bit is trained too.
  val tr_any_we = tr_train.asUInt.orR

  // ── Adaptive training threshold (reference predictor.cc:296-315) ────────────
  // theta walks up on mispredicted bits and down on correct-but-under-confident
  // ones, clamped to the reference's 0..4095. The reference's `tc` counter is
  // reset on every step it takes, so it never accumulates and drops out of the
  // aggregate. Gated with the same init check as the rest of training, since
  // tr_we/per-bit flags are derived from the (uninitialized) weight mems.
  if (useSnipAdaptTheta) {
    val nBad   = PopCount(tr_we)
    val nUnder = PopCount(VecInit((0 until F).map { w => !tr_we(w) && tr_b_under.get(w).asBool }))
    val dTheta = Cat(0.U(1.W), nBad).asSInt -& Cat(0.U(1.W), nUnder).asSInt
    when (tr_b_fire && wt_init_done) {
      val raw = theta.get +& dTheta
      theta.get := Mux(raw > 4095.S, 4095.S, Mux(raw < 0.S, 0.S, raw))
    }
  }
  // val tr_any_we = false.B  // disable training for now, to save power and avoid interference with ITC testing

  // ── ONE muxed write port per mem (init || training), exactly ONE .write() call site each ──
  for (t <- 0 until Tbl) {
    val init_wr = !wt_init_done && (wt_init_idx < E.U)
    val wr_en   = init_wr || (tr_b_fire && tr_any_we)
    val wr_idx  = Mux(init_wr, wt_init_idx(log2Ceil(E)-1, 0), tr_b_idx(t))
    val wr_data = Mux(init_wr, initRow, tr_updated_wt(t))
    when (wr_en) { weights(t).write(wr_idx, wr_data) }
  }
  val bias_init = !wt_init_done
  val bwr_en    = bias_init || (tr_b_fire && tr_any_we)
  val bwr_idx   = Mux(bias_init, wt_init_idx, tr_b_bias_idx)
  val bwr_data  = Mux(bias_init, initRow, tr_updated_bias)
  when (bwr_en) { biasMem.write(bwr_idx, bwr_data) }

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

  // Compare the SNIP target against the full architectural target, and against the
  // baseline the frontend would have consumed without us — the earlier banks'
  // prediction at the committed CFI's column.
  val carried_base_oh  = RegNext(io.update.bits.meta(offBaseOH + bankWidth - 1, offBaseOH))
  val carried_base_all = RegNext(io.update.bits.meta(offBaseTgt + bankWidth * vaddrBitsExtended - 1, offBaseTgt))
  val carried_base_tgt = VecInit((0 until bankWidth).map(w =>
    carried_base_all((w + 1) * vaddrBitsExtended - 1, w * vaddrBitsExtended)))
  val carried_new_target  = RegNext(io.update.bits.meta(offNewTarget + vaddrBitsExtended - 1, offNewTarget))
  val actual_target       = tr_b_target
  val attr_sel            = u.cfi_idx.bits
  val attribution_fire    = tr_b_fire && carried_snip_valid
  val new_correct         = carried_new_target === actual_target
  val base_correct        = carried_base_oh(attr_sel) && (carried_base_tgt(attr_sel) === actual_target)
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
  def itcSet(pc: UInt): UInt = fetchIdx(pc)(itcIdxW - 1, 0)

  // ── Commit-side ITC Update RMW ─────────────────────────────────────────────
  val upd_fire = io.update.valid && u.is_commit_update &&
                 u.cfi_is_jalr && !u.cfi_is_ret && u.cfi_taken && u.cfi_idx.valid
  val upd_idx  = itcSet(u.pc)

  // ITC commit read DROPPED — carried in meta (TAGE pattern). Unpacked as tr_carried_pool.
  // The column index is gone with the address: the write is PC-addressed like the read.
  // The tree-pLRU state is the one exception: it is a pure function of the commit
  // stream, so it is read and written in place at commit rather than riding a
  // predict-time snapshot that a later commit to the same row may have obsoleted.

  val b_fire   = RegNext(upd_fire, false.B)
  val b_idx    = RegNext(upd_idx)
  val b_target = RegNext(u.target)
  val b_ways   = tr_carried_pool

  val b_hit_oh = VecInit(b_ways.map(w => w.valid && w.target === b_target))
  val b_hit    = b_hit_oh.asUInt.orR

  io.itc_total_event := b_fire
  io.itc_hit_event   := b_fire && b_hit

  // Invalid-first, else the tree walk: a free way always wins, so a set that
  // still has one never lets the tree pick the victim (the MRU update still runs).
  val inval_oh  = VecInit(b_ways.map(w => !w.valid))
  val has_inval = inval_oh.asUInt.orR

  val plru_state = RegInit(VecInit(Seq.fill(flatDepth)(0.U(plruNodes.W))))
  val plru_tree  = VecInit(plru_state(itcAddr(b_idx)).asBools)

  // Victim walk: descend the LRU pointers. A node already at a leaf self-loops,
  // so its (unused) bit read is a don't-care.
  val v_node = Wire(Vec(plruL + 1, UInt(plruNodeIdxW.W)))
  v_node(0) := plruRoot.U
  for (l <- 0 until plruL) {
    val bit = plru_tree(plruBitIdxVec(v_node(l)))
    v_node(l + 1) := Mux(bit, plruRightVec(v_node(l)), plruLeftVec(v_node(l)))
  }
  val plru_victim = plruWayVec(v_node(plruL))
  val victim      = Mux(has_inval, PriorityEncoder(inval_oh), plru_victim)

  // MRU update: every node containing the accessed way points away from it,
  // i.e. bit := "the access went left". Nodes off the path keep their bit.
  val acc_way = Mux(b_hit, PriorityEncoder(b_hit_oh), victim)
  // Node bounds reach itc_nWays (the root spans [0, nWays)), so the range tests
  // run one bit wider than the way index itself.
  val acc_cmp = if (plruCmpW > plruWayW) Cat(0.U((plruCmpW - plruWayW).W), acc_way) else acc_way
  val plru_next = Wire(Vec(plruNodes, Bool()))
  for (i <- 0 until plruNodes) {
    val node   = plruBitNode(i)
    val onPath = (acc_cmp >= pLo(node).U(plruCmpW.W)) && (acc_cmp < pHi(node).U(plruCmpW.W))
    plru_next(i) := Mux(onPath, acc_cmp < pMid(node).U(plruCmpW.W), plru_tree(i))
  }
  when (b_fire) { plru_state(itcAddr(b_idx)) := plru_next.asUInt }

  // ── ITC Initialization FSM ─────────────────────────────────────────────────
  val init_done = RegInit(false.B)
  val init_idx  = RegInit(0.U(log2Ceil(flatDepth).W))
  val init_zero = {
    val e = Wire(new ITCEntry); e.valid := false.B; e.target := 0.U; e
  }
  val init_wr = !init_done

  when (!init_done) {
    init_idx := init_idx + 1.U
    when (init_idx === (flatDepth - 1).U) { init_done := true.B }
  }

  for (w <- 0 until itc_nWays) {
    val is_victim = !b_hit && (w.U === victim)
    val is_hitway = b_hit && b_hit_oh(w)
    val commit_we = b_fire && (is_victim || is_hitway)

    val e = Wire(new ITCEntry)
    e.valid  := true.B
    e.target := Mux(is_victim, b_target, b_ways(w).target)

    // Single write call site, Muxed addr/data, no mask → 1R1W → BRAM
    val we    = init_wr || commit_we
    val waddr = Mux(init_wr, init_idx, itcAddr(b_idx))
    val wdata = Mux(init_wr, init_zero, e)
    when (we) { itc(w).write(waddr, wdata) }
  }

  // ── Predict-side ITC Read — one pool for the whole packet ─────────────────
  // The read address comes from io.f0_pc alone: no dependency on any earlier
  // bank's prediction. s3 consumers (below) resolve the column themselves.
  val s3_resp = io.resp_in(0).f3
  val s2_itc_set    = RegNext(RegNext(itcSet(io.f0_pc)))  // double RegNext → s2 stage
  val s2_read_addr  = itcAddr(s2_itc_set)
  val s3_pool = VecInit((0 until itc_nWays).map(w =>
    itc(w).read(s2_read_addr, s2_valid)))

  val s3_taken_mask = VecInit((0 until bankWidth).map(w => s3_resp(w).taken))
  val s3_has_taken = s3_taken_mask.asUInt.orR

  // Fingerprint observer
  io.fp_computed_event := s3_valid && s3_has_taken
  io.fp_nonzero_event  := s3_valid && s3_has_taken && (s3_fingerprint =/= fp_untrained)

  // ── Hamming-match + min-select (s3) ────────────────────────────────────────
  // JALR proxy: is_jal && taken. Already computed at s2, carried via RegNext.
  val cand_fp    = VecInit(s3_pool.map(e => extractFp(e.target)))
  val cand_valid = VecInit(s3_pool.map(e => e.valid))

  val ham = VecInit((0 until itc_nWays).map { w =>
    PopCount((s3_fingerprint ^ cand_fp(w)).asUInt)  // 4 bits, 0..15
  })
  // Key: top bit = !valid (invalid sorts above all valid), lower bits = ham
  val key = VecInit((0 until itc_nWays).map { w =>
    Cat(!cand_valid(w), ham(w))  // 5-bit key
  })

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
  val (snip_sel, _) = reduceMin((0 until itc_nWays).map(w =>
    (w.U(log2Ceil(itc_nWays).W), key(w))))

  val snip_valid   = cand_valid.asUInt.orR && s3_valid
  val snip_target  = s3_pool(snip_sel).target
  val snip_sig     = snip_target(F - 1, 0)
  val snip_min_ham = ham(snip_sel)
  // The override fires whenever the candidate pool has a valid way (snip_valid):
  // the Hamming-distance confidence gate that used to sit here (snip_min_ham <=
  // a configurable threshold, default 2) has been removed, so there is no
  // separate enable signal left. snip_min_ham is still carried in the meta and
  // observable, so what the gate would have rejected can be counted at commit
  // (snip_min_ham_le2/le4); the attribution counters (indirect_*) now measure the
  // ungated override.

  // Step-2: unconditional override of every column, now also unconditional on the
  // prediction's confidence. The frontend only consumes predicted_pc.bits on a
  // column its own decode proves is a JALR (JAL/BR/CFI_X columns take
  // brsigs.target or never redirect), so writing all columns is safe and lets the
  // L3 correct a *wrong* earlier-bank target, not merely fill a hole.
  // Precondition: RAS enabled — a ret column's target is taken from ras_top.
  // NOTE: this also makes frontend f3_btb_mispredicts fire on JAL columns; that
  // path is currently hardwired off (f4_btb_corrections.io.enq.valid := false.B).
  val base_valid_oh = VecInit((0 until bankWidth).map(w =>
    io.resp_in(0).f3(w).predicted_pc.valid))
  val base_tgt_vec  = VecInit((0 until bankWidth).map(w =>
    io.resp_in(0).f3(w).predicted_pc.bits))

  // Carry predict bias + weights + ITC pool + snip fields through f3_meta (TAGE pattern)
  // Layout: under | path_slice | new_target | base targets | base_valid |
  //         pool | wt | bias | valid|min_ham|sig|fingerprint
  // The ring slice is re-aligned s1 → s3 so it is the value the s1-stage index used.
  val s3_path = if (useSnipPathRing) Some(RegNext(RegNext(io.f1_path(W_PATH - 1, 0)))) else None
  // Fields are concatenated MSB-first in offset order, and disabled ones are left
  // out entirely rather than padded: a one-bit zero placeholder between two live
  // fields would shift everything above it out of the slice the composer reads.
  val metaFields: Seq[Bits] =
    (if (useSnipAdaptTheta) Seq(s3_under.get.asUInt) else Nil) ++
    (if (useSnipPathRing)  Seq(s3_path.get)             else Nil) ++
    Seq(snip_target, base_tgt_vec.asUInt, base_valid_oh.asUInt,
        s3_pool.asUInt, s3_wt.asUInt, s3_bia.asUInt,
        snip_valid, snip_min_ham, snip_sig, s3_fingerprint)
  io.f3_meta := metaFields.reduceLeft((a, b) => Cat(a, b))

  // F3 target override: every column, whenever the candidate pool is non-empty.
  when (snip_valid) {
    for (w <- 0 until bankWidth) {
      io.resp.f3(w).predicted_pc.valid := true.B
      io.resp.f3(w).predicted_pc.bits  := snip_target
    }
  }

  // All-column observers still dropped (the read yields one pool, not a per-column view)
  io.pred_taken_event          := s3_valid && s3_has_taken
  io.pred_pool_nonempty_event  := false.B
  io.pred_target_in_pool_event := false.B
  io.pred_pool_saturated_event := false.B
}
