# OpenHumanoid Motion Compiler

> Compile human motion into validated, simulator-ready trajectories for different humanoid robots.

[![CI](https://github.com/Ethan-Schenberg/openhumanoid-motion-compiler/actions/workflows/ci.yml/badge.svg)](https://github.com/Ethan-Schenberg/openhumanoid-motion-compiler/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)
[![Python](https://img.shields.io/badge/python-3.10%2B-blue.svg)](pyproject.toml)

OpenHumanoid Motion Compiler (OHMC) is a proposed open-source ROS 2 motion-retargeting stack. It treats motion conversion as a compiler problem: human-motion sources are parsed into a robot-independent intermediate representation, transformed by explicit constraint passes, and emitted through simulator or robot-specific backends.

The project is currently in the **design and prototype stage**. Unitree and AgiBot X2 are first-class target platforms. Their official SDKs, robot models, simulators, and ROS 2 interfaces are integrated through pinned vendor manifests and dedicated adapters. No real-robot execution command is enabled by default in v0.1.

## Why this project

Most humanoid motion demos tightly couple one input format, one robot model, one inverse-kinematics implementation, and one control interface. That makes the result difficult to inspect, reproduce, or port.

OHMC separates those concerns:

```text
Human video / BVH / mocap
            |
            v
     Source frontends
            |
            v
     Canonical skeleton
            |
            v
       Motion IR
            |
            v
 Constraint and optimization passes
            |
            v
 MuJoCo / ros2_control / Unitree / AgiBot X2
```

Its core deliverable is not a single dance. It is a reviewable compilation pipeline with deterministic inputs, explicit transformations, validation reports, and reproducible outputs.

## v0.1 scope

The first milestone is deliberately offline and simulation-only:

- Import BVH motion.
- Load a redistributable humanoid URDF.
- Map canonical human joints to robot joints through an explicit profile.
- Solve constrained whole-body inverse kinematics.
- Enforce joint, velocity, acceleration, and contact constraints.
- Export a versioned Motion IR artifact and validation report.
- Replay the result in MuJoCo.
- Resolve and verify pinned Unitree and AgiBot X2 development dependencies.
- Build vendor-adapter interface tests without requiring physical hardware.
- Reproduce the same output from the same inputs and configuration.

Not included in v0.1:

- Commands to physical robots.
- Unlicensed redistribution of vendor artifacts.
- Learned controllers or sim-to-real claims.
- Automatic safety certification.
- Video-to-3D pose estimation.

## Design principles

1. **Robot-independent core** — robot-specific knowledge belongs in declarative profiles or backend adapters.
2. **First-class vendor support** — Unitree and AgiBot X2 adapters, SDK acquisition, version locks, and compatibility tests live in the main project rather than in unofficial downstream forks.
3. **Simulation before hardware** — every motion must pass offline validation and replay before a hardware adapter can consume it.
4. **Explicit semantics** — frame conventions, joint mappings, contacts, limits, and assumptions are stored with the artifact.
5. **Inspectable passes** — each transformation produces metrics and diagnostics rather than silently modifying trajectories.
6. **Deterministic builds** — pinned inputs and configuration should produce equivalent Motion IR outputs.
7. **License-aware dependency delivery** — permissively licensed SDKs may be pinned as source dependencies; official downloadable SDKs without a complete redistribution license are acquired and checksum-verified by tooling rather than copied into Git history.

## Documents

- [Project white paper](docs/WHITEPAPER.md)
- [v0.1 architecture](docs/ARCHITECTURE_V0.1.md)
- [Vendor SDK integration policy](docs/VENDOR_SDKS.md)
- [Offline robot profiles](docs/ROBOT_PROFILES.md)
- [Maintainers and governance](MAINTAINERS.md)

## Development quick start

Requires Python 3.10 or newer:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e '.[dev,mujoco]'
```

Run the complete offline pipeline with one command. This smoke target uses a
small synthetic MuJoCo model while still exercising the Unitree G1 profile and
non-executable adapter contract:

```bash
.venv/bin/ohmc simulate examples/simple_motion.bvh \
  --target unitree-g1-contract-fixture \
  --source-license CC0-1.0 \
  --source-convention right_handed_x_right_y_up_z_backward \
  --source-length-unit m \
  --output build/one-click-smoke
```

The command emits an atomic, hash-addressed evidence bundle containing
canonical skeleton motion, source and mapped Motion IR, derived velocity and
acceleration, a trajectory-quality report, copied resolved configuration, a
replay report, a vendor interface fixture, and `manifest.json`. IK-enabled
targets also contain the resolved task map, solver-neutral IK problem, and
per-frame IK result. It refuses to overwrite an existing build directory.

After resolving the pinned vendor dependencies, the same command replays
against the official Unitree G1 29DoF or AgiBot X2 Ultra MuJoCo model:

```bash
.venv/bin/ohmc vendor sync unitree
.venv/bin/ohmc simulate examples/simple_motion.bvh \
  --target unitree-g1 \
  --source-license CC0-1.0 \
  --source-convention right_handed_x_right_y_up_z_backward \
  --source-length-unit m \
  --output build/unitree-g1 \
  --cache-dir .ohmc-cache

.venv/bin/ohmc vendor import agibot-x2 /path/to/X2_URDF-v1.3.0.zip
.venv/bin/ohmc simulate examples/simple_motion.bvh \
  --target agibot-x2-ultra \
  --source-license CC0-1.0 \
  --source-convention right_handed_x_right_y_up_z_backward \
  --source-length-unit m \
  --output build/agibot-x2-ultra \
  --cache-dir .ohmc-cache

# Larger bilateral legs, waist, and arms benchmark (still labelled partial IK)
.venv/bin/ohmc simulate examples/full_body_motion.bvh \
  --target unitree-g1-multilimb-benchmark \
  --source-license CC0-1.0 \
  --source-convention right_handed_x_forward_y_left_z_up \
  --source-length-unit m \
  --output build/unitree-g1-multilimb \
  --cache-dir .ohmc-cache

.venv/bin/ohmc simulate examples/full_body_motion.bvh \
  --target agibot-x2-ultra-multilimb-benchmark \
  --source-license CC0-1.0 \
  --source-convention right_handed_x_forward_y_left_z_up \
  --source-length-unit m \
  --output build/agibot-x2-multilimb \
  --cache-dir .ohmc-cache
```

These are headless kinematic `mj_forward` replays. The official targets now run
a constrained partial-body IK proof on each vendor model, while the manifest
still explicitly marks whole-body IK, closed-loop dynamics, and hardware
transport as unavailable. See [the IK contract](docs/IK_CONTRACT.md) and
[landmark coverage contract](docs/LANDMARK_COVERAGE.md), plus the
[project roadmap](docs/ROADMAP.md), for the exact boundary and acceptance gates.

Validate the reference Motion IR artifact:

```bash
.venv/bin/ohmc validate-ir examples/minimal_motion.json
```

Inspect the synthetic CC0 BVH fixture and import its ordered rotation channels
into a prototype Motion IR artifact:

```bash
.venv/bin/ohmc inspect-source examples/simple_motion.bvh
.venv/bin/ohmc canonicalize-bvh examples/simple_motion.bvh \
  --source-license CC0-1.0 \
  --source-convention right_handed_x_right_y_up_z_backward \
  --source-length-unit m \
  --output build/canonical-motion.json
.venv/bin/ohmc normalize-canonical build/canonical-motion.json \
  --rate-hz 100 \
  --morphology-scale 0.9 \
  --output build/canonical-motion-normalized.json
.venv/bin/ohmc retarget-ik build/canonical-motion-normalized.json \
  --robot profiles/unitree_g1_29dof.yaml \
  --task-map examples/ik_task_map_fixture.yaml \
  --model examples/ik_contract_fixture.xml \
  --output build/ik-contract
.venv/bin/ohmc landmark-report build/canonical-motion-normalized.json \
  --task-map examples/ik_task_map_fixture.yaml \
  --output build/landmark-coverage.json
.venv/bin/ohmc import-bvh examples/simple_motion.bvh \
  --source-license CC0-1.0 \
  --output build/simple_motion.json
.venv/bin/ohmc derive-kinematics build/simple_motion.json \
  --output build/simple_motion_kinematics.json
.venv/bin/ohmc validate-ir build/simple_motion.json
```

The canonicalizer evaluates joint offsets, position channels, and rotation
channels in declared order; converts source axes and units into right-handed
`+X` forward, `+Y` left, `+Z` up metres; and emits local quaternions plus world
joint poses. BVH files do not reliably declare axes or length units, so both
inputs are mandatory instead of guessed. See
[the canonical motion contract](docs/CANONICAL_MOTION.md).

The normalization pass preserves local joint translations, rescales skeleton
morphology, resamples rotations with shortest-arc quaternion SLERP, preserves
the exact source endpoint, and recomputes every world pose with forward
kinematics. One-command simulation targets pin both normalization rate and
morphology scale, and bundle the source and normalized canonical artifacts.

The separate Motion IR importer converts rotation values from degrees to
radians and records the source hash, timing, channel order, configuration hash,
skipped translation channels, and the fact that this prototype joint-vector
path has not consumed canonical transforms yet. It is deliberately labelled
with the `bvh_rotation_channels_v1` prototype profile: it is not yet an
IK-retargeted robot trajectory and cannot command hardware.

After semantic mapping, generate the machine-readable quality report:

```bash
.venv/bin/ohmc quality-report build/unitree_g1_motion.json \
  --robot profiles/unitree_g1_29dof.yaml \
  --output build/unitree_g1_quality.json
```

The report distinguishes actual violations, missing dynamic limits, and
incomplete joint coverage. `--require-complete-mapping` and
`--require-dynamic-limits` make those missing guarantees fail the command while
preserving the report. See [trajectory quality gates](docs/QUALITY_GATES.md).

Install the optional MuJoCo dependency and run a headless kinematic replay:

```bash
.venv/bin/python -m pip install -e '.[mujoco]'
.venv/bin/ohmc replay build/simple_motion.json \
  --backend mujoco \
  --model examples/simple_chain.xml \
  --report build/replay-report.json
```

The current backend maps Motion IR joints by exact name, enforces MuJoCo scalar
joint limits, applies each sample, and runs `mj_forward` to reject non-finite
kinematics. It does not open a viewer, simulate a controller, connect to ROS 2,
or send hardware commands.

Validate the two vendor-backed offline robot profiles and map the same source
artifact into their declared joint contracts:

```bash
.venv/bin/ohmc inspect-robot profiles/unitree_g1_29dof.yaml
.venv/bin/ohmc inspect-robot profiles/agibot_x2_ultra_aimdk_v1.yaml

.venv/bin/ohmc map-joints build/simple_motion_kinematics.json \
  --robot profiles/unitree_g1_29dof.yaml \
  --mapping profiles/mappings/simple_bvh_semantics_v1.yaml \
  --output build/unitree_g1_motion.json

.venv/bin/ohmc map-joints build/simple_motion_kinematics.json \
  --robot profiles/agibot_x2_ultra_aimdk_v1.yaml \
  --mapping profiles/mappings/simple_bvh_semantics_v1.yaml \
  --output build/agibot_x2_motion.json

.venv/bin/ohmc encode-fixture build/unitree_g1_motion.json \
  --robot profiles/unitree_g1_29dof.yaml \
  --adapter unitree-g1-lowcmd \
  --output build/unitree_g1_lowcmd_fixture.json

.venv/bin/ohmc encode-fixture build/agibot_x2_motion.json \
  --robot profiles/agibot_x2_ultra_aimdk_v1.yaml \
  --adapter agibot-x2-joint-command-array \
  --output build/agibot_x2_joint_command_fixture.json
```

Both profiles set `hardware_transport: disabled`. The current mapping example
demonstrates deterministic joint ordering, sign conversion, and model-limit
checks. The fixture encoders preserve the official `LowCmd` and
`JointCommandArray` field/order contracts while leaving modes, gains, effort,
damping, headers, and missing positions unset. They do not serialize middleware
messages, publish topics, perform whole-body IK, or create a physical execution
path. See [the fixture contract](docs/INTERFACE_FIXTURES.md).

Inspect and verify vendor SDK dependencies:

```bash
.venv/bin/ohmc vendor status
.venv/bin/ohmc vendor verify
.venv/bin/ohmc vendor doctor
```

Synchronize the pinned Unitree SDK, ROS 2, and MuJoCo repositories:

```bash
.venv/bin/ohmc vendor sync unitree
```

Import official AgiBot X2 downloads into the verified local cache:

```bash
.venv/bin/ohmc vendor import agibot-x2 /path/to/aimdk-aarch64-artifacts.zip
.venv/bin/ohmc vendor import agibot-x2 /path/to/X2_URDF-v1.3.0.zip
.venv/bin/ohmc vendor verify agibot-x2
.venv/bin/ohmc vendor doctor agibot-x2 --json
```

The dependency cache defaults to `~/.cache/ohmc`. Set `OHMC_CACHE_DIR` or pass `--cache-dir` to use another location. Cached SDKs are not committed to the OHMC repository.

Run the test suite:

```bash
.venv/bin/python -m pytest
```

None of these commands connect to or command a physical robot.

## Proposed repository layout

```text
ohmc/
  frontends/
  ir/
  passes/
  solvers/
  validators/
  backends/
  vendors/
    unitree/
    agibot_x2/
  robot_profiles/
  cli/
docs/
examples/
schemas/
tests/
vendor/
  vendor-lock.yaml
```

## Project status

Implemented foundation:

- Installable Python package and `ohmc` command.
- Motion IR v0.1 JSON Schema plus semantic validation.
- Valid reference Motion IR fixture.
- Vendor lock parsing and compatibility status.
- SHA-256 verified import for official AgiBot X2 artifacts.
- Pinned Git synchronization for Unitree SDK2, ROS 2, and MuJoCo repositories.
- Automated tests for Motion IR semantics and artifact integrity.
- Strict BVH hierarchy/channel/frame parsing plus deterministic rotation-channel
  import into schema-valid Motion IR.
- Optional headless MuJoCo joint mapping and kinematic replay validation with a
  machine-readable report.
- Schema-validated Unitree G1 29DoF and AgiBot X2 Ultra offline profiles with
  explicit joint ordering, limits, grouping, and exclusions.
- Non-executable Unitree G1 `LowCmd` and AgiBot X2 `JointCommandArray` interface
  fixtures with schema validation and CI conformance tests.
- One-command, atomic simulation evidence bundles for synthetic contract smoke
  tests and pinned official Unitree G1/AgiBot X2 Ultra models.
- Canonical BVH skeleton evaluation with explicit source axes/units, ordered
  local rotations, metre offsets, quaternions, and deterministic world poses.
- Non-uniform timestamp velocity/acceleration derivation, profile limit checks,
  and explicit per-robot mapping-completeness reports with strict CLI gates.
- Uniform morphology scaling, exact-duration resampling, shortest-arc SLERP,
  FK recomputation, and pass-level input/output hash chaining.
- Solver-neutral IK task/problem/result schemas plus a bounded deterministic DLS
  reference solver with explicit per-frame failures, residuals, and active
  joint limits.
- Partial-body IK integrated into official G1 and X2 evidence bundles without
  claiming whole-body coverage.
- A 16-landmark original CC0 full-body BVH benchmark and machine-readable source
  and IK-task coverage reports with strict CLI gates.

Full bilateral task coverage, orientation/contact constraints, dynamic
controller playback, and rendered comparison video are the next implementation
milestones.
Real-robot execution remains outside v0.1.
