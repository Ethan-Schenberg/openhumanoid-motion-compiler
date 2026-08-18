# OHMC North-Star Roadmap

Status: active roadmap

North star: licensed human motion to reproducible dual-vendor simulation evidence

## The higher target

OHMC should become the command-line compiler and benchmark harness that turns a
licensed BVH motion into reviewable simulation results for both Unitree G1 and
AgiBot X2 Ultra without manual file editing:

```text
licensed BVH
    -> canonical skeleton and full-body constrained IK
    -> Unitree G1 + AgiBot X2 target matrix
    -> contact-aware closed-loop MuJoCo replay
    -> side-by-side video, metrics, artifacts, and provenance
```

The v1.0 experience should be one command from a fresh Ubuntu environment:

```bash
ohmc simulate motion.bvh --target all --render --output build/run
```

That command must never imply physical-robot authority. Simulation, ROS 2
message conformance, and hardware validation remain separate evidence levels.

## Current project audit

The repository already has a sound foundation: versioned Motion IR, strict
schemas, deterministic BVH ingestion, two vendor profiles, pinned dependency
verification, offline adapter fixtures, and headless MuJoCo replay. The new
`ohmc simulate` command now composes those pieces into an atomic evidence
bundle and resolves official models from the verified vendor cache.

Trajectory derivatives and quality reports now expose position/dynamic-limit
violations, missing limits, and exact controllable-joint coverage. Strict CLI
gates can reject incomplete mapping or dynamic-limit evidence.

The largest technical gaps are not packaging gaps. They are motion-quality and
validation gaps:

- The internal DLS solver is a deterministic reference, not the intended
  production IK framework. Mink/GMR parity and adapter work now precedes new
  solver features.
- Replay is kinematic `mj_forward`, not actuator/controller dynamics.
- Foot contacts, balance, and self-collision are not yet enforced. Velocity and
  acceleration are derived and checked wherever profiles declare limits;
  missing limits remain explicit warnings.
- No rendered comparison video or benchmark dashboard exists.
- Vendor interface fixtures are structural JSON contracts; the real SDK/ROS 2
  message adapters do not yet compile in the CI matrix.

## Milestones and acceptance gates

### v0.2 — One-command evidence bundle (implemented)

- `ohmc simulate` performs import, semantic mapping, validation, MuJoCo replay,
  and vendor fixture encoding.
- A schema-validated manifest records input, model, configuration, and artifact
  SHA-256 values.
- Output is atomic and existing build directories are never overwritten.
- Synthetic smoke and official Unitree/AgiBot model targets are distinct.

### v0.3 — Canonical motion compiler (in progress)

- Convert BVH local transforms into the documented right-handed canonical
  skeleton. Implemented with explicit source convention/unit, ordered Euler
  evaluation, local quaternions, and world-pose golden tests.
- Add morphology scaling and deterministic timeline resampling passes.
  Implemented with preserved local translations, shortest-arc quaternion SLERP,
  exact-duration endpoint policy, and world-pose FK recomputation.
- Emit pass-level input/output hashes and metrics. Implemented for BVH
  canonicalization and morphology/timeline normalization.
- Gate: mirrored, rotated, and different-height fixtures produce stable expected
  canonical coordinates on Python 3.10–3.12.
- An original CC0 16-landmark/51-channel full-body BVH fixture and strict source
  coverage report are implemented; richer morphology variants remain pending.

### v0.4 — Full-body constrained IK

- Solver-neutral task/problem/result schemas and a deterministic bounded DLS
  MuJoCo reference solver are implemented.
- G1 and X2 official-model smoke bundles execute a three-variable left-hip
  position task with explicit per-frame residuals and failure states.
- The larger multi-limb benchmark now consumes all 16 canonical landmarks as
  position tasks and drives all 29 G1 and all 30 available X2 command variables.
  Both official models solve all three CC0 benchmark frames below 5 mm, and a
  strict target gate rejects incomplete source or task coverage.
- Full landmark-position coverage remains labelled partial-body IK: the G1
  head task is a torso proxy, and orientation, contact, balance, and collision
  constraints are not implemented.
- Solve all configured legs, waist, arms, and available head joints.
- Enforce profile position limits and report residuals per frame.
- Add foot-contact preservation, self-collision checks, smoothing, and explicit
  failure states without silent fallback.
- Gate: every benchmark frame is either solved or explicitly failed; no NaN,
  hidden clamp, or unreported limit violation is accepted.

### v0.5 — Dynamic simulation and regression metrics

- Trajectory-level peak velocity, acceleration, jerk, and minimum position
  margin are implemented with responsible-joint attribution and aggregate
  consistency checks. Missing vendor limits remain warnings.
- Add position-controller and actuator-aware MuJoCo replay.
- Measure foot slip, base drift, joint-limit margin, peak velocity,
  acceleration, torque proxy, and contact discontinuity.
- Add deterministic regression thresholds for G1 and X2.
- Gate: the same benchmark produces equivalent metrics in a clean Linux CI run;
  a threshold regression fails the build.

### v0.6 — Visual evidence and target matrix

- Implement `--target all` and isolated per-target failure reporting.
  Implemented with benchmark-family selection, atomic child bundles, child
  manifest hashes, preserved failures, and semantic matrix validation.
- Render source skeleton and both robot results side by side.
- Export MP4, plots, and a compact HTML report from the same manifest.
- Gate: one command produces a reviewable comparison with provenance and no
  network access after dependencies are cached.

### v0.7 — Real SDK and ROS 2 conformance

- Compile Unitree SDK2/ROS 2 and AgiBot AimDK adapters in controlled container or
  vendor-compatible build jobs.
- Serialize recorded-message fixtures and verify field order, units, timestamps,
  QoS assumptions, and unavailable X2 head-pitch handling.
- Gate: adapter compilation and recorded-message tests pass without a robot and
  without enabling transport.

### v1.0 — Reproducible humanoid motion benchmark

- Publish licensed benchmark motions and pinned dual-vendor models.
- Provide a clean-environment installer, cached/offline mode, migration policy,
  performance budget, and signed release artifacts.
- Gate: a third party can reproduce the published bundles, videos, and metrics
  using only documented commands.
- Independent deterministic bundle/matrix integrity verification is implemented
  for manifests, safe paths, child bundles, and artifact SHA-256 values.

## Immediate contribution queue

The next implementation order is:

1. Add a Mink backend spike behind the solver-neutral contract. Compare it with
   the internal DLS oracle on both official models for residual, limit,
   determinism, runtime, and explicit failure parity.
2. Add a GMR adapter for licensed BVH input and G1 output, then evaluate an X2
   configuration as an upstream contribution rather than a private fork.
3. Define Motion IR interchange fixtures for ProtoMotions and one maintained
   downstream learning framework; do not implement another training stack.
4. Add frame-orientation and contact tasks through the selected mature backend,
   followed by actuator-aware replay metrics.
5. Produce side-by-side rendered regression evidence from the same manifest.

All major queue items follow the [research-before-build policy](RESEARCH_POLICY.md)
and the decisions in [the upstream adoption map](UPSTREAM_ADOPTION.md). The
project will invest in contracts, adapters, cross-vendor coverage, and evidence
rather than duplicating mature IK, physics, training, or ROS 2 control stacks.
