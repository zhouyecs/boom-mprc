package boom.v3.ifu

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import boom.v3.common._

// ITTAGE indirect-target predictor (Seznec-style, no base table).
//   - 4 tagged tables keep the low 20 bits of the target (index = pc ^ folded history,
//     geometric history lengths; partial tag disambiguates entries).
//   - 1 fully-associative 128-entry region array keeps the high 20 bits (tree-pLRU).
// Prediction (s2 read -> s3 resolve): provider = longest-history table with a tag hit;
// the provider's tag looks up the region array; target = {region_high, table_low};
// overrides io.resp.f3(col).predicted_pc exactly like SNIP/BLBP.
// Training (commit-side, TAGE-pattern): the 4 read entries ride through the fetch
// pipeline in f3_meta so commit needs no re-read; indices/tags are recomputed from
// commit pc/ghist. Provider hit: correct => u++, wrong => u-- + low rewrite.
// Mispredict: allocate in the lowest-history table longer than the provider with an
// invalid or useless (u==0) entry. Region array: in-place high update on tag match,
// else allocate invalid-first then tree-pLRU victim.
class IttageBranchPredictorBank(implicit p: Parameters) extends BranchPredictorBank()(p) {
  // ── Geometry ──────────────────────────────────────────────────────────────
  val T        = 4                               // tagged tables, increasing histLens
  val nEnt     = 8192                            // entries per table (folded depth: nRows x bankWidth cols)
  val tagW     = 7                               // partial tag bits
  val lowW     = 20                              // low target bits kept in tables
  val highW    = 20                              // high target bits kept in region array
  val nRegions = 128                             // region array entries (fully associative)
  require(nEnt % bankWidth == 0, s"nEnt ($nEnt) must be a multiple of bankWidth ($bankWidth)")
  val nRows = nEnt / bankWidth                   // hashed rows per table
  val idxW  = log2Ceil(nRows)                    // hash index width
  val colW  = log2Ceil(bankWidth)

  val histLens = Seq(8, 16, 32, 64).map(_ min globalHistoryLength)
  require(histLens.length == T)

  // ── Entries ───────────────────────────────────────────────────────────────
  class TableEntry extends Bundle {
    val valid = Bool()
    val tag   = UInt(tagW.W)
    val low   = UInt(lowW.W)
    val u     = UInt(2.W)                        // useful counter
  }
  val W_E = 1 + tagW + lowW + 2                  // 30

  // meta layout (LSB-first): 4 carried table entries, 30 bits each
  val offE = Seq.tabulate(T)(t => t * W_E)
  override val metaSz = T * W_E                  // 120
  require(metaSz <= bpdMaxMetaLength,
    s"ITTAGE metaSz ($metaSz) exceeds bpdMaxMetaLength ($bpdMaxMetaLength)")

  // Folded Cat(row,col) index — full-entry write (no mask) → BRAM-compatible
  val tables = Seq.fill(T) { SyncReadMem(nEnt, UInt(W_E.W)) }
  def foldAddr(idx: UInt, col: UInt): UInt =
    if (bankWidth == 1) idx else Cat(idx(idxW - 1, 0), col(colW - 1, 0))

  // Region array: valid/rtag as regs (CAM), high bits in async Mem (s3 read + commit write)
  val region_high  = Mem(nRegions, UInt(highW.W))
  val region_valid = RegInit(VecInit(Seq.fill(nRegions)(false.B)))
  val region_rtag  = RegInit(VecInit(Seq.fill(nRegions)(0.U(tagW.W))))

  // Tree-pLRU state (127 nodes, shared tree; invalid-first on allocation)
  val L      = log2Ceil(nRegions)
  val nNodes = nRegions - 1
  val plru_state = RegInit(VecInit(Seq.fill(nNodes)(false.B)))

  println(s"[ITTAGE cfg] T=$T nEnt=$nEnt nRows=$nRows idxW=$idxW tagW=$tagW lowW=$lowW " +
    s"nRegions=$nRegions histLens=${histLens.mkString(",")} " +
    s"metaSz=$metaSz/$bpdMaxMetaLength bankWidth=$bankWidth")

  // ── Hashing (tage.scala compute_tag_and_hash scheme) ─────────────────────
  def foldHist(hist: UInt, len: Int, chunkW: Int): UInt = {
    if (len == 0) 0.U(chunkW.W)
    else {
      val h = hist(len - 1, 0)
      val nChunks = (len + chunkW - 1) / chunkW
      val chunks = (0 until nChunks).map { i =>
        h(Math.min((i + 1) * chunkW, len) - 1, i * chunkW)
      }
      chunks.reduce(_ ^ _)
    }
  }
  def hashIdx(pc: UInt, ghist: UInt, t: Int): UInt =
    (fetchIdx(pc) ^ foldHist(ghist, histLens(t), idxW))(idxW - 1, 0)
  def hashTag(pc: UInt, ghist: UInt, t: Int): UInt =
    ((fetchIdx(pc) >> idxW) ^ foldHist(ghist, histLens(t), tagW))(tagW - 1, 0)

  // ── Init FSMs (tables + region in parallel; predict/update gated) ─────────
  val init_done = RegInit(false.B)
  val init_idx  = RegInit(0.U(log2Ceil(nEnt).W))
  when (!init_done) {
    init_idx := init_idx + 1.U
    when (init_idx === (nEnt - 1).U) { init_done := true.B }
  }

  val region_init_done = RegInit(false.B)
  val region_init_idx  = RegInit(0.U(log2Ceil(nRegions).W))
  when (!region_init_done) {
    region_init_idx := region_init_idx + 1.U
    when (region_init_idx === (nRegions - 1).U) { region_init_done := true.B }
  }

  // ── Predict side (s2 read → s3 provider + region CAM) ─────────────────────
  val s2_resp = io.resp_in(0).f2
  val s2_jalr_mask = VecInit((0 until bankWidth).map(w =>
    s2_resp(w).is_jal && s2_resp(w).taken))
  val s2_snip_col  = PriorityEncoder(s2_jalr_mask)

  val s2_ghist = RegNext(io.f1_ghist)            // f1 → s2
  val s2_idx_t = VecInit((0 until T).map(t => hashIdx(s2_idx, s2_ghist, t)))
  val s2_tag_t = VecInit((0 until T).map(t => hashTag(s2_idx, s2_ghist, t)))

  val readEn = s2_valid && init_done
  val s3_entries = VecInit((0 until T).map(t =>
    tables(t).read(foldAddr(s2_idx_t(t), s2_snip_col), readEn).asTypeOf(new TableEntry)))

  // Carry s2 col/jalr/tag info to s3
  val snip_col    = RegNext(s2_snip_col)
  val s3_has_jalr = RegNext(s2_jalr_mask.asUInt.orR)
  val s3_tag_t    = RegNext(s2_tag_t)
  val s3_init_ok  = RegNext(init_done)           // sampled at s2 (readEn vintage)

  val s3_resp = io.resp_in(0).f3
  val s3_taken_mask = VecInit((0 until bankWidth).map(w => s3_resp(w).taken))
  val s3_has_taken = s3_taken_mask.asUInt.orR

  // Tag-compare at s3; provider = longest-history hit (last hit wins, tage.scala idiom)
  val hits = VecInit((0 until T).map(t =>
    s3_entries(t).valid && s3_entries(t).tag === s3_tag_t(t)))
  var provider = 0.U
  for (t <- 0 until T) {
    provider = Mux(hits(t), t.U, provider)
  }
  val provider_hit = hits.asUInt.orR

  // Region lookup: CAM provider tag over regs, async-read high bits
  val region_match   = VecInit((0 until nRegions).map(i =>
    region_valid(i) && region_rtag(i) === s3_entries(provider).tag))
  val region_hit     = region_match.asUInt.orR
  val region_win     = PriorityEncoder(region_match)
  val region_high_s3 = region_high.read(region_win)

  val ittage_target = Cat(region_high_s3, s3_entries(provider).low)
  val snip_override = s3_init_ok && s3_valid && s3_has_jalr && provider_hit && region_hit
  when (snip_override) {
    io.resp.f3(snip_col).predicted_pc.valid := true.B
    io.resp.f3(snip_col).predicted_pc.bits  := ittage_target
  }

  // ── f3_meta carry (TAGE pattern; commit unpacks, no re-read) ──────────────
  io.f3_meta := Cat((0 until T).reverse.map(t => s3_entries(t).asUInt))

  // ── Commit-side training ──────────────────────────────────────────────────
  val u       = io.update.bits
  val tr_fire = io.update.valid && u.is_commit_update &&
                u.cfi_is_jalr && !u.cfi_is_ret && u.cfi_taken && u.cfi_idx.valid

  val tr_b_fire    = RegNext(tr_fire, false.B)
  val tr_b_target  = RegNext(u.target)
  val tr_b_mispred = RegNext(u.cfi_mispredicted)
  val tr_b_col     = RegNext(u.cfi_idx.bits)
  val tr_b_idx_t   = RegInit(VecInit(Seq.fill(T)(0.U(idxW.W))))
  val tr_b_tag_t   = RegInit(VecInit(Seq.fill(T)(0.U(tagW.W))))
  val tr_b_entries = RegNext(io.update.bits.meta(metaSz - 1, 0))

  when (tr_fire) {
    tr_b_idx_t := VecInit((0 until T).map(t => hashIdx(fetchIdx(u.pc), u.ghist, t)))
    tr_b_tag_t := VecInit((0 until T).map(t => hashTag(fetchIdx(u.pc), u.ghist, t)))
  }

  def carriedEntry(t: Int): TableEntry =
    tr_b_entries(offE(t) + W_E - 1, offE(t)).asTypeOf(new TableEntry)

  val actual_low  = tr_b_target(lowW - 1, 0)
  val actual_high = tr_b_target(vaddrBitsExtended - 1, lowW)

  // Commit-time hit detection on recomputed tags (self-consistent with predict)
  val commit_hits = VecInit((0 until T).map(t => {
    val e = carriedEntry(t)
    e.valid && e.tag === tr_b_tag_t(t)
  }))
  var commit_prov = 0.U
  for (t <- 0 until T) {
    commit_prov = Mux(commit_hits(t), t.U, commit_prov)
  }
  val commit_prov_hit = commit_hits.asUInt.orR
  // Provider entry by dynamic index (Mux-tree over the 4 carried entries)
  val prov_entry      = VecInit((0 until T).map(t =>
    tr_b_entries(offE(t) + W_E - 1, offE(t)).asTypeOf(new TableEntry)))(commit_prov)
  val prov_low        = prov_entry.low
  val prov_correct    = commit_prov_hit && (prov_low === actual_low)
  val prov_wrong      = commit_prov_hit && (prov_low =/= actual_low)

  // Allocation (classic TAGE): lowest-history table LONGER than the provider with
  // an invalid or useless (u==0) entry; with no provider, any such table.
  val alloc_mask = VecInit((0 until T).map(t => {
    val cand    = !carriedEntry(t).valid || carriedEntry(t).u === 0.U
    val longerT = t.U > commit_prov
    Mux(commit_prov_hit, cand && longerT, cand)
  }))
  val alloc_slot = PriorityEncoder(alloc_mask)
  val alloc_en   = tr_b_fire && tr_b_mispred && alloc_mask.asUInt.orR

  // Per-table write (single muxed port per mem: init || training)
  for (t <- 0 until T) {
    val is_prov  = commit_prov_hit && (commit_prov === t.U)
    val is_alloc = alloc_en && (alloc_slot === t.U)
    val e = Wire(new TableEntry)
    e.valid := true.B
    e.tag   := tr_b_tag_t(t)
    e.low   := Mux(is_alloc || (is_prov && prov_wrong), actual_low, carriedEntry(t).low)
    val u_inc = Mux(carriedEntry(t).u === 3.U, 3.U, carriedEntry(t).u + 1.U)
    val u_dec = Mux(carriedEntry(t).u === 0.U, 0.U, carriedEntry(t).u - 1.U)
    e.u     := Mux(is_alloc, 0.U, Mux(prov_wrong, u_dec, u_inc))

    val we      = is_prov || is_alloc
    val init_wr = !init_done
    when (init_wr || (tr_b_fire && we)) {
      tables(t).write(
        Mux(init_wr, init_idx, foldAddr(tr_b_idx_t(t), tr_b_col)),
        Mux(init_wr, 0.U, e.asUInt))
    }
  }

  // ── Region array commit update (tag CAM, in-place high update, tree-pLRU) ──
  // Key = tag of the table that will provide future predictions (an allocated
  // longer-history table takes over on mispredict; else the provider).
  val upd_tag = Mux(alloc_en, tr_b_tag_t(alloc_slot),
                Mux(commit_prov_hit, tr_b_tag_t(commit_prov), 0.U))
  val region_upd_en = tr_b_fire && region_init_done && (commit_prov_hit || alloc_en)

  val cmatch    = VecInit((0 until nRegions).map(i =>
    region_valid(i) && region_rtag(i) === upd_tag))
  val cam_hit   = cmatch.asUInt.orR
  val inval_oh  = VecInit(region_valid.map(!_))
  val has_inval = inval_oh.asUInt.orR

  // Victim: descend tree following LRU pointers (BLBP tree-pLRU recurrence)
  val v_node = Wire(Vec(L + 1, UInt(log2Ceil(nNodes + 1).W)))
  val v_way  = Wire(Vec(L + 1, UInt(L.W)))
  v_node(0) := 0.U
  v_way(0)  := 0.U
  for (l <- 0 until L) {
    val bit = plru_state(v_node(l))
    v_way(l + 1)  := (v_way(l) << 1) | bit
    v_node(l + 1) := (v_node(l) << 1) + 1.U + bit
  }
  val plru_victim = Mux(has_inval, PriorityEncoder(inval_oh), v_way(L))
  val wslot       = Mux(cam_hit, PriorityEncoder(cmatch), plru_victim)

  // MRU update: descend to accessed way, flip path bits to point away
  val acc_way = wslot
  val u_node  = Wire(Vec(L + 1, UInt(log2Ceil(nNodes + 1).W)))
  u_node(0) := 0.U
  for (l <- 0 until L) {
    val bit = acc_way(L - 1 - l)                 // MSB first
    u_node(l + 1) := (u_node(l) << 1) + 1.U + bit
  }
  val plru_next = Wire(Vec(nNodes, Bool()))
  for (i <- 0 until nNodes) {
    val onPath  = VecInit((0 until L).map(l => u_node(l) === i.U)).asUInt.orR
    val levelOf = PriorityEncoder(VecInit((0 until L).map(l => u_node(l) === i.U)))
    plru_next(i) := Mux(onPath, !acc_way((L - 1).U - levelOf), plru_state(i))
  }

  // Single call sites: Mem write port + valid/rtag regs muxed with region init
  when (!region_init_done || region_upd_en) {
    val wr_idx = Mux(!region_init_done, region_init_idx, wslot)
    region_high.write(wr_idx, Mux(!region_init_done, 0.U, actual_high))
    region_valid(wr_idx) := Mux(!region_init_done, false.B, true.B)
    region_rtag(wr_idx)  := Mux(!region_init_done, 0.U, upd_tag)
  }
  when (!region_init_done) {
    plru_state.foreach(_ := false.B)
  } .elsewhen (region_upd_en) {
    plru_state := plru_next
  }

  // ── Observers (BranchPredictorBank event conventions) ─────────────────────
  io.itc_total_event := tr_b_fire
  io.itc_hit_event   := tr_b_fire && commit_prov_hit
  io.tr_event        := tr_b_fire
  io.tr_exact_event  := tr_b_fire && prov_correct
  io.pred_taken_event          := s3_valid && s3_has_taken
  io.pred_pool_nonempty_event  := s3_valid && s3_has_jalr && provider_hit
  io.pred_target_in_pool_event := s3_valid && s3_has_jalr && provider_hit && region_hit
  io.pred_pool_saturated_event := false.B

  override val mems =
    Seq.tabulate(T)(t => (s"ittage_table$t", nEnt, W_E)) :+
    ("ittage_region", nRegions, highW)

  // ── Simulation-only diagnostics ───────────────────────────────────────────
  if (IN_SIMULATION) {
    val ovr_cnt = RegInit(0.U(8.W))
    when (snip_override && ovr_cnt < 32.U) {
      ovr_cnt := ovr_cnt + 1.U
      printf(p"[ittage] OVR col=${snip_col} prov=${provider} " +
        p"low=0x${Hexadecimal(s3_entries(provider).low)} " +
        p"high=0x${Hexadecimal(region_high_s3)} tgt=0x${Hexadecimal(ittage_target)}\n")
    }
    val upd_cnt = RegInit(0.U(8.W))
    when (tr_b_fire && upd_cnt < 32.U) {
      upd_cnt := upd_cnt + 1.U
      printf(p"[ittage] UPD hit=${commit_prov_hit} prov=${commit_prov} corr=${prov_correct} " +
        p"alloc=${alloc_en} slot=${alloc_slot} cam_hit=${cam_hit} wslot=${wslot} " +
        p"low=0x${Hexadecimal(actual_low)} high=0x${Hexadecimal(actual_high)}\n")
    }
  }
}
