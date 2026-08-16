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
- [Maintainers and governance](MAINTAINERS.md)

## Development quick start

Requires Python 3.10 or newer:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e '.[dev]'
```

Validate the reference Motion IR artifact:

```bash
.venv/bin/ohmc validate-ir examples/minimal_motion.json
```

Inspect the synthetic CC0 BVH fixture and import its ordered rotation channels
into a prototype Motion IR artifact:

```bash
.venv/bin/ohmc inspect-source examples/simple_motion.bvh
.venv/bin/ohmc import-bvh examples/simple_motion.bvh \
  --source-license CC0-1.0 \
  --output build/simple_motion.json
.venv/bin/ohmc validate-ir build/simple_motion.json
```

The importer converts BVH rotation values from degrees to radians and records
the source hash, timing, channel order, configuration hash, skipped translation
channels, and the fact that canonical axis remapping is still pending. This
ingestion artifact is deliberately labelled with the
`bvh_rotation_channels_v1` prototype profile: it is not yet an IK-retargeted
robot trajectory and cannot command hardware.

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

Inspect and verify vendor SDK dependencies:

```bash
.venv/bin/ohmc vendor status
.venv/bin/ohmc vendor verify
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

Canonical root transforms, robot profiles, constrained IK compiler passes, and
dynamic simulator playback are the next implementation milestones. Real-robot
execution remains outside v0.1.
