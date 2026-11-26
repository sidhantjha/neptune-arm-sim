# HANDOFF — Neptune Transition (temporary home)

**Checkpoint date:** 2026-11-26
**Repository storing this handoff:** `sidhantjha/neptune-arm-sim` on `main`
**Authoritative Neptune repository (not accessible through the current GitHub connector):** `Project-Lumyn/Project-Neptune`
**Full transition brief:** [docs/NEPTUNE_TRANSITION_HANDOFF.md](docs/NEPTUNE_TRANSITION_HANDOFF.md)
**Codex read-only audit prompt:** [docs/CODEX_AUDIT_PROMPT.md](docs/CODEX_AUDIT_PROMPT.md)

## Read this first when the user says `resume project`

1. Fetch/verify the latest `main` for the **actual** `Project-Lumyn/Project-Neptune` repository. Confirm its remote, branch, HEAD SHA and local working-tree status. **Do not substitute either older `sidhantjha/project-neptune` or `sidhantjha/project_neptune` checkout.** The former team repository was returning 404 through the connected GitHub app at checkout. If inaccessible, ask the user for repository access or their current local checkout and do not pretend it was read.
2. In the authoritative Neptune repository, read root `AGENTS.md`, `PROJECT_STATE.md`, `ROADMAP.md`, `ACTIVE_WORK.md`, `DECISIONS.md`, `EXPERIMENTS.md`, `HANDOFF.md` in full; read `docs/PROJECT_HANDOFF.md`, `docs/ARCHITECTURE.md`, `docs/INTERFACES.md`, `docs/TEAM_CONTRACTS.md`, `docs/OWNERSHIP.md`, and `docs/PROJECT_HISTORY.md` where present. Newer source and continuity state supersedes this temporary handoff.
3. Check the current Neptune roadmap phase, active workstream, last accomplishment, ownership, outstanding blockers, and exact next action. Reconcile any differences before changes.
4. Start a **read-only Codex codebase audit** using [the prepared prompt](docs/CODEX_AUDIT_PROMPT.md). No architecture reorganization or code modification before inventory, verified working baseline, and explicit refactor plan.
5. Report the current phase, exact active workstream, last verified accomplishment, blocker (if any), and the next exact action; then proceed one safe step at a time with the user.

## What is VERIFIED right now
- The standalone educational `neptune-arm-sim` repository's M0–M4 scope is complete, with PyBullet 2R arm, quintic joint trajectories, parallel-jaw gripper, frictional grasp and autonomous known-pose pick-and-place. On user's Windows/Conda environment: **M2 6/6, M3 6/6, M4 6/6 PASS**. Headless and GUI M4 demos completed all task states; example placement error approximately **0.0204 m**; this is not evidence of real underwater performance.
- M4 code merged to arm-sim `main` at `cf80ff7ab025da0d3256926a3fb84a8ccbb52d01`; arm-sim docs checkpoint before this handoff at `900dd3ed88789147a70159374091b8f0ac374ee4`.
- A user screenshot shows **`Small_scale_sub_2.stp`** (STEP, **4.9 MB**, uploaded/updated Nov 24, 2026) in Box folder **Project Lumyn / Talon / sub_CAD**. Contents, joint structure, whether the arm is included, physical parameters and export suitability have **not** been inspected.
- The connected GitHub app could not access the designated canonical `Project-Lumyn/Project-Neptune` repository (404 at checkout); older personal repositories were found but their state **must not** be treated as current team `main`.

## Current user goal and next exact action
The user wants a **clean, teachable, modular, extensible, easily debuggable** Neptune ROS 2/Stonefish stack leading to a simulation of a submarine holding station by a pipe and using an articulated arm/gripper to pick up an object on or near the pipe. The generic vehicle + generic 2-DOF arm should be implemented first while CAD is assessed, so autonomy is not blocked by mechanics.

**Next exact action: identify and open the user's actual local canonical Neptune checkout, verify `origin` / remote `main` / HEAD / cleanliness, read the canonical context, and run a no-edits Codex audit.** Only afterward design and review a minimal refactor with baseline tests.

## Planned following 5 steps — not yet performed
1. Capture Codex audit: package structure, nodes, topics, launch graph, ROS/Stonefish version and environment, dependencies, tests, rendering/timing problems, verified/unverified claims, and debugging commands.
2. Reproduce current baseline end-to-end mission; record build, tests, launch commands, demo metrics, simulator/ROS outputs and known failures without changing semantics.
3. Agree on a minimal modular refactor on a feature branch, preserving existing `neptune_*` package boundaries, public interfaces and ownership; migrate a small slice, test, checkpoint.
4. Model a **generic AUV + 2-DOF arm** in Stonefish, test actuator/arm states and coupled buoyancy/trim/thruster behavior while existing navigation retains its canonical estimated-state input.
5. Inspect provided STEP CAD; obtain separated visual/physical meshes, joint axes, frames and mass/hydrostatic properties, then compare the real geometry to the generic baseline. Later build pipe-object grasp and quantify stability and mission success.

## Non-negotiable Neptune contracts
- `/state/odometry`: canonical navigation state. `/sim/odometry`: evaluation/debugging truth ONLY; no production autonomy/controls use.
- Mission executive: sole normal publisher of `/neptune/cmd_body`; `neptune_vehicle` owns body-command→actuator mapping.
- Vehicle body frame: +X forward/bow, +Y starboard, +Z **down**; positive `linear.z` means dive. Educational arm PyBullet world Z points **up**, so transforms cannot be copied unchanged.
- GUI is read-only for vehicle motion. Changes to public topics, types, signs, frames, units, semantics or ownership require `docs/INTERFACES.md` and test/cross-module review.
- Preserve teammate work. Do not force push `main`; substantive implementation belongs on a feature branch.
- Current handoff is planning and documentation only: **no parent Neptune code, local checkout, CAD geometry or Stonefish simulation changed at this checkpoint**.

## User preference
User learns by inspecting real small modules and writing code incrementally. Explain Python functions, classes, objects, `self`, initialization, `return`, controller `update(dt)`, and state-machine transitions in short concrete examples as encountered. Keep designs straightforward rather than introducing abstract frameworks for their own sake.

## Commands
**To close a chat:** `end chat` or `check out`. Do continuity updates and only claim push after verifying it.

**To resume in a new chat:** `resume project`. Read this handoff, then verify and read the authoritative **Project Neptune** repo. Do not assume the arm-sim repo is the parent project's source of truth.
