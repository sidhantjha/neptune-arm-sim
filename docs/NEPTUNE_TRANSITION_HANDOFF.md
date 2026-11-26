# Project Neptune — Engineering Transition Handoff

**Saved:** 2026-11-26. **Temporary storage:** [neptune-arm-sim](https://github.com/sidhantjha/neptune-arm-sim).
**Authority:** [Project-Lumyn/Project-Neptune](https://github.com/Project-Lumyn/Project-Neptune) when its current `main` becomes accessible. This is an interim continuation record, **not a verified snapshot of the current canonical Neptune tree**.
**Entry point:** [../HANDOFF.md](../HANDOFF.md). **Codex audit prompt:** [CODEX_AUDIT_PROMPT.md](CODEX_AUDIT_PROMPT.md).

## 1. Mission, user goals, and rationale

Build a full working underwater autonomous demonstration for Talon/Neptune:
- vehicle starts and navigates toward a subsea pipe/work asset, approaches and holds useful standoff/station,
- 2-DOF articulated manipulator with gripper moves, grasps an object located on or near the pipe, and retrieves/relocates it as the target demo is finalized,
- monitor whether arm movement disturbs depth, position, pitch/yaw/roll and whether thrusters/controllers compensate,
- later replace stand-in geometry with the team's real submarine + arm CAD and recalibrate physics based on measured real vehicle properties.

The immediate desire is **maintainable software that new engineers can clone, run, understand, debug and extend**. No huge one-shot rewrite. Reuse good ROS 2/Stonefish conventions: single responsibility per node/module, stable interfaces, simulation-vs-hardware boundary, pure/testable calculation code, clear ROS entry points, documented configs, deterministic reproducible tests, and a functioning baseline before each refactor.

The user wants to understand the actual code and write parts themselves. They found Python class/method/`self`/`update(dt)` semantics and large state machines harder than FK/IK and gripper physics. Teach these concepts as they appear; prefer practical small increments and clear verification steps rather than massive auto-generated code dumps.

## 2. Verified standalone arm-simulator accomplishments

Repository: `sidhantjha/neptune-arm-sim`; completed code on main `cf80ff7`, documentation checkpoint `900dd3e`. The existing repository's `README.md`, `docs/PROJECT.md`, `docs/PROGRESS.md`, `docs/EXPERIMENTS.md` and `docs/NEXT_STEPS.md` are detailed validated sources; **do not overwrite these history files to pretend the parent Neptune integration is complete**.

- M0/M1: Windows Conda `neptune-arm` (Python 3.11) and PyBullet, fixed 2R X-Z arm.
- M2: analytical forward/inverse kinematics, two IK branches, quintic interpolation, stateful `ArmController` with `_plan_joint_goal()` saving start/goal/duration and `update(dt)` commanding incremental motor positions; **6/6 PASS**.
- M3: five-joint URDF including two revolute arm axes, fixed palm and two prismatic fingers, physical contact and friction grasp; 40 mm cube, 30/50/80 g repeated trials; **6/6 PASS**.
- M4: known-sim-pose pick-and-place task, states `HOME → OPEN → APPROACH → ALIGN → GRASP → LIFT → TRANSFER → PLACE → RELEASE → RETREAT → SUCCESS`. Headless and GUI demo succeeded. Repeated six-trial suite at 30/50/80 g: **6/6 PASS**. Sample cube final location `(0.4696,0.0016,0.6000) m` vs goal `(0.49,0,0.60) m` → error **0.0204 m**. The 0.345 m gripper-center reach was compared to PyBullet geometry; error ~`0.00000004 m` at tested pose.
- Limitations: **fixed-base**, planar, known cube pose, fixed hard-coded transfer waypoints, no independent wrist, no general collision-free planner, no underwater coupling, no camera-based object pose, no actual Talon actuation or real force calibration.
- Tests are user-reported Windows console results, not independent assistant test execution.

## 3. Canonical Neptune repository access: BLOCKER

User-defined authoritative repository: `https://github.com/Project-Lumyn/Project-Neptune.git`. At checkout 2026-11-26, connected GitHub read returned **404**, so neither the latest team `main` nor the seven root continuity files could be inspected, and we cannot truthfully give a canonical current HEAD, worktree status, dependencies, active roadmap or current package state.

Two **historical/personal** repos did appear accessible:
- `sidhantjha/project-neptune` (`main` `dd8ea5a`, dated Sep 2026, thin Stage 0 docs).
- `sidhantjha/project_neptune` (`main` `1d568cf`, dated Oct 2026, Stage 1 sensor-derived mission baseline; root AGENTS.md and docs/PROJECT_HANDOFF.md).
Their histories offer **leads, not truth about team main**. Never clone/edit them in place of the canonical repo without explicit confirmation from the user.

The older Oct 2026 snapshot described a ROS 2 workspace with packages `neptune_interfaces`, `neptune_state_estimation`, `neptune_perception`, `neptune_world`, `neptune_autonomy`, `neptune_controls`, `neptune_vehicle`, `neptune_evaluation`, `neptune_sim`, `neptune_bringup`; a sensor-derived mission `WAIT_FOR_STATE → DIVE → TRANSIT → INSPECT → RETURN → ASCEND → COMPLETE`; Stonefish integration; historic WSLg DVL callback stalls. **These are historical claims and may have changed.** Confirm against current team code before using or refactoring.

### Canonical contract-level invariants to preserve
- `/state/odometry` is the navigation/controller state; `/sim/odometry` is simulation truth for evaluation/debugging only.
- Mission executive sole normal publisher of `/neptune/cmd_body`; `neptune_vehicle` owns downstream body-command→actuator translation.
- Vehicle body +X bow, +Y starboard, +Z down; positive `linear.z` dive. PyBullet standalone arm +Z up—frame transform required.
- GUI read-only for vehicle motion.
- Any change to topic, message, frame, units, sign, ownership or semantics requires `docs/INTERFACES.md`, affected tests and module-owner review.
- Preserve current team work, feature branch for substantive changes, never force push main.

## 4. CAD status and physical model strategy

Screenshot from Box: `Project Lumyn / Talon / sub_CAD / Small_scale_sub_2.stp`, **4.9 MB**, shown as updated **Nov 24, 2026**. **No CAD download, parser inspection or assembly validation was performed.** This may be hull only or include arm; the screenshot does not prove joint hierarchy or motor design. Avoid committing large CAD or generated meshes to source control without agreement.

The right staged plan:
1. **Generic AUV + generic 2R manipulator** mounted in Stonefish, using separate rigid bodies/joints rather than a single merged STL; preserve the current autonomy baseline. Evaluate arm/vehicle coupling and station-hold during arm motion.
2. In parallel inspect source STEP in a CAD tool and request: editable native/STEP assembly, independent rigid-link definitions, joint pivots/axes/limits, base mounting transform, visual meshes and simplified closed collision/physics meshes (generally OBJ/ASCII STL per Stonefish setup), hull sealed displacement envelope, mass/COM/inertia per body, thruster and sensor mounting frames.
3. Replace stand-in geometries/physical parameters with verified CAD and real-world values **incrementally**. Initially visual-only to compare alignment if prudent; then collision, hydrostatics, inertias, actuator model and hydrodynamics in controlled steps.
4. Real vehicle parameters will differ from CAD. Calibrate: weigh assembly, measure buoyancy/displacement, CG/trim, thruster thrust curves, actuator torque/speed; identify drag/added mass from measured motion; compare real/Stonefish experiments; retune controllers only after physical parameter estimation.
5. Explicitly report which parameters are measured vs CAD-estimated vs tuned assumptions, with uncertainty/ranges and repeatable configuration.

Stonefish physics gives a model-based engineering estimate of coupling and loads; it is **not proof of actual underwater performance**, nor a replacement for motor sizing, structural checks, or field measurements.

## 5. Suggested development phases (not yet started)

**A. Read-only inventory + environment:** verify canonical repo remote/main/HEAD/cleanliness and actual roadmap/docs. Have Codex map every ROS package/node/launch/config, module owners, topics/types, Stonefish scenario and thrusters, build/reproducibility, WSL/Docker/GUI, known rendering or timing issues, tests and metrics. No changes; request file/path-backed evidence.

**B. Establish repeatable baseline:** clone/pull in correct Ubuntu/WSL ROS 2 environment, capture `colcon` build/test outputs, run the existing mission, log topics/status and current simulation behavior. Identify actual setup errors (OpenGL/WSLg problems and callback gaps are only *historical possibilities*). Commit concise SETUP/DEBUGGING guidance after verification.

**C. Minimal architectural cleanup:** preserve current `neptune_*` packages and cross-package ROS interfaces. Extract small pure functions, dedicated configuration, simulation model setup, controller/kinematics utilities, typed input/output, parameter validation and unit tests **only where code inspection shows need**. Refactor one module/slice per change and prove equivalence with tests and metrics; don't build an overengineered global `utils` package or rewrite working autonomy for style.

**D. Generic Stonefish AUV + 2R arm:** add articulated body/joints, arm state observation and safe actuator commands behind clear module boundaries. Demonstrate reliable floating and arm motion with arm stowed/extended at station-hold; quantify vehicle drift, pitch/roll, thruster saturation and controller effort. Do not change core autonomy topics or use `/sim/odometry` as feedback.

**E. Pipe-object interaction:** fixture/pipe and target object, approach/standoff/hold, arm reach and safe actuation, grasp/contact, lift/retrieve, retreat/return. Define success, collision margin and stability thresholds before tuning. Introduce controlled pose/perception estimates rather than silently depending on simulator truth.

**F. Import Talon CAD and compare:** same scenarios with actual hull/arm geometry and calibrated physical parameters, capture differences to generic results, adjust model and controllers separately.

These phases are a **proposal** to reconcile against the parent Neptune roadmap; do not assume M5 is the canonical team priority without checking `ACTIVE_WORK.md`.

## 6. Exact resume procedure

At the next chat, user can say **`resume project`**. Assistant must first:
1. Identify the canonical team checkout; user's local project path is unknown and must not be invented. Check `git rev-parse --show-toplevel`, `git remote -v`, `git status`, `git branch --show-current`, `git fetch origin`, and `git rev-parse origin/main`. Protect uncommitted changes; no `git switch main` or `git pull` blindly if dirty/diverged. When clean and on main, use `git pull --ff-only origin main`.
2. Read all root `AGENTS.md`, `PROJECT_STATE.md`, `ROADMAP.md`, `ACTIVE_WORK.md`, `DECISIONS.md`, `EXPERIMENTS.md`, `HANDOFF.md`; and detailed `docs/PROJECT_HANDOFF.md`, `docs/ARCHITECTURE.md`, `docs/INTERFACES.md`, `docs/TEAM_CONTRACTS.md`, `docs/OWNERSHIP.md`, `docs/PROJECT_HISTORY.md` if present. If absent, record which are missing; **do not silently create an alternative history**.
3. Run the [read-only Codex audit prompt](CODEX_AUDIT_PROMPT.md), or ask user to run locally and paste report. If local Codex isn't available, inspect source directly in read-only mode; do not insist on one tool.
4. State current roadmap phase, active workstream, last verified accomplishment, blocker and exact next action. Review audit and choose a minimal first change, with owner review if cross-module.

## 7. Open issues and assumptions
- Real Neptune team repo access/HEAD and branch status: unknown, highest-priority blocker.
- True current Stonefish setup, scenario XML layout, ROS distro, Docker/WSL run commands, rendering failures, sensor topic map, collision meshes and test coverage: unknown until audit.
- Does STEP contain all arm links? Can CAD export separated watertight mesh parts? Vehicle displacement, CG/CB, mass/inertia, thruster mounts, actuator specs: unknown.
- Demo target: pipe-top object grasp is **proposed**, not implemented; target object type/size, arm workspace, vehicle standoff, final retrieval action and evaluation tolerances remain design decisions.
- Avoid equating a deterministic 6/6 PyBullet result to underwater effectiveness.
