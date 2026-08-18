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

- Current mapping covers named rotation channels, not canonical full-body IK.
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
- Emit pass-level input/output hashes and metrics.
- Gate: mirrored, rotated, and different-height fixtures produce stable expected
  canonical coordinates on Python 3.10–3.12.

### v0.4 — Full-body constrained IK

- Solve all configured legs, waist, arms, and available head joints.
- Enforce profile position limits and report residuals per frame.
- Add foot-contact preservation, self-collision checks, smoothing, and explicit
  failure states without silent fallback.
- Gate: every benchmark frame is either solved or explicitly failed; no NaN,
  hidden clamp, or unreported limit violation is accepted.

### v0.5 — Dynamic simulation and regression metrics

- Add position-controller and actuator-aware MuJoCo replay.
- Measure foot slip, base drift, joint-limit margin, peak velocity,
  acceleration, torque proxy, and contact discontinuity.
- Add deterministic regression thresholds for G1 and X2.
- Gate: the same benchmark produces equivalent metrics in a clean Linux CI run;
  a threshold regression fails the build.

### v0.6 — Visual evidence and target matrix

- Implement `--target all` and isolated per-target failure reporting.
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

## Immediate contribution queue

The next implementation order is:

1. Morphology scaling and deterministic timeline resampling.
2. Solver-neutral IK problem contract and a small deterministic reference
   solver.
3. Canonical landmark-to-robot task mapping for full-body IK.
4. Contact-aware dynamic replay metrics.
5. Side-by-side rendered regression evidence.

This order improves the mathematical core before adding presentation layers,
so future rendered demos remain auditable rather than becoming disconnected
showcases.
