# Codex Prompt — Project Neptune Read-Only Architecture, Environment, and Reliability Audit

**Purpose:** Copy this entire prompt into Codex while it is opened at the **actual** `Project-Lumyn/Project-Neptune` repository root on the current, verified checkout. **Do not run it inside the older personal repos or inside `neptune-arm-sim`.**

```text
You are auditing the existing Project Neptune ROS 2 + Stonefish robotics codebase.

CONSTRAINTS — READ ONLY:
- Do NOT edit, create, rename, or delete any repository files.
- Do NOT commit, push, switch branches, reset, stash, install dependencies,
  run a script that modifies the workspace, launch a full simulator, or run
  long-running processes as part of this initial audit.
- You may run lightweight READ-ONLY commands (pwd, git status, git log,
  git remote -v, find, rg, sed, cat, inspecting package manifests, etc.).
- Do not assume facts that cannot be verified; distinguish VERIFIED from
  INFERRED and UNKNOWN.
- No implementation, refactor, or architecture proposal should be silently
  executed. Return a report and stop.

FIRST: identify checkout and current project truth
1. Show exact repository path, git remotes, current branch, HEAD SHA,
   remote main reference if already fetched, and git working-tree state.
2. Verify this is Project-Lumyn/Project-Neptune, not either older personal repo.
3. Read in full the root AGENTS.md, PROJECT_STATE.md, ROADMAP.md,
   ACTIVE_WORK.md, DECISIONS.md, EXPERIMENTS.md, HANDOFF.md, where present.
4. Read relevant detailed documents:
   docs/PROJECT_HANDOFF.md, docs/ARCHITECTURE.md,
   docs/INTERFACES.md, docs/TEAM_CONTRACTS.md,
   docs/OWNERSHIP.md, docs/PROJECT_HISTORY.md, README.md.
5. Report missing documents and conflicting claims explicitly.
   Repository state supersedes any previous conversation or handoff.

SECOND: map the implemented architecture and actual runtime
6. Draw an ASCII tree of meaningful source folders and ROS 2 packages.
   Do not dump build/install/log caches, recordings or generated artifacts.
7. For EACH package, identify:
   - purpose and owner if documented,
   - source entrypoints, executable/node names and key classes,
   - publishers/subscribers/services/actions with exact topics and types,
   - major parameters, configs and dependencies,
   - tests and significant TODOs.
   Cite file paths and line numbers where practical.
8. Trace the actual end-to-end runtime:
   Stonefish scenario -> simulated sensors -> estimation -> /state/odometry
   -> mission executive -> guidance/controls -> /neptune/cmd_body
   -> neptune_vehicle -> actuators.
   Also trace evaluation truth flow and confirm /sim/odometry does NOT
   close the production control loop.
9. Inventory existing Stonefish scenario XML, meshes, actuator definitions,
   joint models, thrusters, sensor frames, simulation step and timing,
   ROS bridges, launch/bringup order, real hardware adapter pathways.
10. Identify whether there are *already* arm, manipulator, pipe, object,
    grasp, hold-station, or vehicle-CAD assets. Do not guess.
11. Identify logic duplicated across files and overly large scripts,
    unstable interfaces, state-machine edge cases, hidden assumptions,
    misaligned frame conventions, tightly coupled simulation calls,
    and untestable ROS callbacks, with concrete examples.

THIRD: reproducibility and debug audit
12. Determine supported OS / Ubuntu / ROS 2 distro, Python version,
    Docker image/compose, conda/venv if relevant, dependencies,
    build and run commands from repo. Do not invent commands.
13. Determine how to reproduce the *current working mission* and
    minimum isolated smoke tests. Give safe commands separately from
    commands already executed, and note expected outputs.
14. Look for documented or code-evident rendering failures
    (OpenGL / WSLg / GPU / GUI), startup/shutdown failures,
    DVL/IMU/depth timing gaps, ROS timing/sim-time handling,
    transform mistakes, blocking callbacks, stale topics,
    log spam or crashes. Historical issues may have been fixed:
    separate code-confirmed vs old-document-only statements.
15. Inspect test/CI/lint infrastructure and benchmark/log collection.
    Identify critical missing tests before any refactor.

FOURTH: evaluate modularity against mature robotics engineering principles
16. Prefer existing ROS 2 package boundaries and clear interfaces, with
    small pure math/config utilities, small ROS node adapters, parameterized
    scenario models, single publisher ownership and hardware abstraction.
    Avoid gratuitous classes, an unbounded utils package and massive rewrite.
17. Propose 3 possible levels of change:
    A. minimal cleanup with no public interface change,
    B. moderate selective refactor with compatibility tests,
    C. new AUV+generic 2-DOF arm Stonefish functionality.
    For each state affected files, risk, owner, tests, rollback strategy.
18. Preserve hard contracts unless explicit cross-module review approves:
    /state/odometry = estimated vehicle state,
    /sim/odometry = evaluation/debug truth ONLY,
    mission executive = normal publisher of /neptune/cmd_body,
    neptune_vehicle = body command to actuator translation,
    body frame +X bow/+Y starboard/+Z down;
    positive linear.z = dive; GUI cannot command vehicle motion.

FINAL: deliver one concise but thorough AUDIT REPORT with sections:
A. Checkout identity + git safety / known blockers
B. Current roadmap phase, active workstream and last verified accomplishment
C. Architecture diagram, package and executable table
D. ROS topics/frames/units/ownership contract table
E. End-to-end startup and control/data flow
F. Stonefish integration/sensors/thrusters/vehicle geometry inventory
G. Reproducible setup/run/test commands (mark actually verified or not)
H. Issues and risks with severity, file references and evidence
I. Minimal refactor plan in dependency order (NO implementation yet)
J. Generic AUV + 2DOF arm integration roadmap, CAD import readiness
K. Exact first next action plus 3–5 steps, each with acceptance criteria.

Project goal: modular, readable, debug-friendly underwater autonomy that
eventually demonstrates the vehicle stabilizing by a pipe and picking up an
object via an articulated arm. Start with generic AUV+generic arm while
actual CAD is evaluated. Real mass, buoyancy, inertia and thruster response
must later be calibrated from measurements; simulation values are estimates.

For this audit, produce a report only; do not write any files.
```

## Recommended local commands, once the correct checkout is identified

```powershell
git rev-parse --show-toplevel
git remote -v
git branch --show-current
git status
git fetch origin
git rev-parse origin/main
```

Do not discard uncommitted work or assume the directory is safe to overwrite. For a clean local checkout on `main`, a safe update is `git pull --ff-only origin main`; if behind/diverged/dirty, reconcile first. Then open VS Code at the repository root and run Codex in read-only mode if supported by the local installation (for example `codex --sandbox read-only`). If the environment is WSL2/Ubuntu, perform ROS/Stonefish actions in that environment.

After the audit, the user should share the report and the exact repository SHA. The next assistant must read the repo's authoritative continuity files and review the report before proposing concrete edits.
