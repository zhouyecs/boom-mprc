# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a fork of [BOOM](https://boom-core.org) (Berkeley Out-of-Order Machine), a synthesizable, parameterizable RV64GC RISC-V core written in Chisel. This fork focuses on **Return Address Stack (RAS)** microarchitecture research, adding dual-stack RAS designs with call compression.

The core is a Chisel generator — it produces Verilog, not a standalone build. It is used as a submodule within [Chipyard](https://chipyard.readthedocs.io/), which provides the SoC integration, simulation, and PPA flows. The active development branch is `dev_yz_sqras`.

## BOOM v3 vs v4

- **v3** (`src/main/scala/v3/`): SonicBOOM — the stable, high-performance microarchitecture. **All RAS research is in v3.** The full `BoomRasStack` module exists only here.
- **v4** (`src/main/scala/v4/`): Next-generation BOOM. Has basic RAS support (ras_idx, ras_top in FTQ entries) but does NOT instantiate `BoomRasStack`. Not relevant for RAS work.

## Source Layout

```
src/main/scala/
├── v3/
│   ├── common/
│   │   ├── parameters.scala      # BoomCoreParams, HasBoomCoreParameters, HasBoomFrontendParameters
│   │   ├── config-mixins.scala   # WithNMediumBooms, WithMySQRASMediumBooms, etc.
│   │   ├── consts.scala          # Pipeline constants (BSRC_*, IQT_*)
│   │   ├── package.scala         # Type aliases, helper functions
│   │   └── tile.scala            # BoomTile wrapper
│   ├── ifu/                      # Instruction Fetch Unit (frontend)
│   │   ├── frontend.scala        # Main frontend pipeline (F0-F4), RAS instantiation (~1100 lines)
│   │   ├── bpd/
│   │   │   ├── ras.scala         # BoomRasStack — THE core RAS module
│   │   │   ├── predictor.scala   # BranchPredictor, BranchPredictorBank, prediction bundles
│   │   │   ├── composer.scala    # ComposedBranchPredictorBank — chains BPD components
│   │   │   ├── tage.scala        # TAGE predictor
│   │   │   ├── btb.scala         # Branch Target Buffer
│   │   │   ├── bim.scala / hbim.scala / local.scala / loop.scala / ubtb.scala / tourney.scala / faubtb.scala
│   │   │   └── sw_predictor.scala
│   │   ├── fetch-target-queue.scala  # FTQ — stores prediction metadata, drives redirect recovery
│   │   └── icache.scala / fetch-buffer.scala
│   ├── exu/                      # Execution units
│   ├── lsu/                      # Load/store unit
│   └── util/                     # Utility modules (CircularQueuePtr, etc.)
└── v4/                           # Next-gen BOOM (not relevant for RAS)
```

## RAS Architecture

### Dual-structure design

The `BoomRasStack` (v3/ifu/bpd/ras.scala) is a standalone module — it is NOT part of the BPD component composition. It's instantiated directly in the frontend (`frontend.scala:349`).

```
Commit Stack (committedReturnAddressStack)
  └── Non-speculative, updated only at retirement (commit)
  └── Size: rasCommitStackSize

Speculative Queue (speculativeReturnAddressQueue)
  └── Circular buffer, written speculatively at prediction time
  └── Size: rasSpecQueueSize
  └── Linked-list via specNextOnStack for NOS (next-on-stack) pointer traversal

Pointers:
  - committedReturnAddressStackPtr  — commit stack top
  - specStackPtr / specCounter      — speculative view of commit stack state
  - specReadPtr / specWritePtr      — speculative queue head/tail (circular)
  - specBasePtr                     — oldest still-in-flight queue entry (for overflow detection)
```

### Call compression

Each `RasEntry` has both a `returnAddr` and a `counter`. When a call pushes an address that matches the current top, the counter increments instead of allocating a new slot. When popping, the counter decrements first; only when it reaches 0 does the stack pointer move. Counter width = `rasCounterWidth` (default 3, max count = 7).

### Write bypass + predictedTop

For timing closure, writes to the speculative queue are delayed by one cycle (`delayedPush`). A write bypass structure (`bypassEntry`/`bypassValid`) provides the most recent write during the delay. The `predictedTop` register pre-computes the next-cycle top-of-stack using the bypass and delayed-push values, so the prediction output (`io.spec.popAddr`) is available combinatorially.

### RAS meta and FTQ snapshots

Each FTQ entry captures a `RasMeta` snapshot containing: `specStackPtr`, `specCounter`, `specReadPtr`, `specWritePtr`, `nextOnStack`. On a misprediction redirect, the RAS restores from this snapshot via `io.redirect.meta`, then re-applies the redirect call/ret.

### Integration in the frontend pipeline

```
F0: NextPC → ICache request, BPD request
F1: ICache access, TLB translation
F2: ICache response, BPD f2 prediction
F3: Decode, RAS push/pop/predict, instruction classification (call/ret/jalr/br)
F4: Send to ID stage
```

At F3, when `f3_fire` is true:
- Calls trigger `ras.io.spec.pushValid` → speculative push
- Returns trigger `ras.io.spec.popValid` → speculative pop, `ras.io.spec.popAddr` drives the predicted target
- Near-overflow (`ras.io.specNearOverflow`) inhibits push/pop to prevent queue overflow

RAS commit updates come from the FTQ at retirement (`ftq.io.ras_commit`). Redirects come from the FTQ on misprediction (`ftq.io.ras_redirect`).

## BPD Component Composition

Branch prediction components are composed via `getBPDComponents()` in `HasBoomFrontendParameters` (parameters.scala:254), which calls `boomParams.branchPredictor(resp_in, p)`. This function is a `BoomCoreParams` field set by config mixins (e.g., `WithTAGEBPD` sets it to `TAGEBranchPredictorBank`). `ComposedBranchPredictorBank` chains components: each component's `resp_in` is the previous component's output. Meta bits are concatenated in order, LSB first.

RAS is NOT chained through this composition — it's a separate module.

## Key Parameters (BoomCoreParams)

| Parameter | Default | Description |
|---|---|---|
| `numRasEntries` | 32 | Legacy parameter (kept >0 to enable `useRAS`) |
| `rasCommitStackSize` | 16 | Commit stack depth |
| `rasSpecQueueSize` | 32 | Speculative queue depth |
| `rasCounterWidth` | 3 | Call compression counter bits |
| `enableRasTopRepair` | true | TOS repair on misprediction |
| `branchPredictor` | `(Nil, resp_in)` | Function composing BPD components |
| `bpdMaxMetaLength` | 120 | Max meta bits across all BPD components |

## Code Style

A `scalastyle-config.xml` is at the repo root. Run style checks:

```bash
make checkstyle
```

Or directly: `sbt scalastyle test:scalastyle`

## Debug Flags

Defined in `frontend.scala` (~line 1112) as `PlusArg`:

| Flag | Effect |
|---|---|
| `+ras-printf=1` | General RAS push/pop/redirect events |
| `+ras-content=1` | Commit stack content dump |
| `+ras-sq-content=1` | Speculative queue content dump |
| `+debug-ret-frontend=1` | Frontend return PC, RAS target, offset |
| `+debug-ret-backend=1` | Backend return PC, actual target, offset |

Pass these as simulator arguments: `./simulator-<CONFIG> +ras-printf=1 ...`

## Build & Test (from chipyard root)

See the chipyard root CLAUDE.md for full build/sim/PPA commands. Quick reference:

```bash
# In sims/verilator/
make CONFIG=MediumBoomV3Config -j$(nproc)
make CONFIG=SQRASx8y4z2 -j$(nproc)   # requires WithMySQRASMediumBooms config

# In tests/
cmake -S ./ -B ./build/ -D CMAKE_BUILD_TYPE=Debug
cmake --build ./build/ --target test_ras
```
