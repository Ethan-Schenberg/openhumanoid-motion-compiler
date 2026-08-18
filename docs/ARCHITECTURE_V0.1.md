# OHMC v0.1 Architecture

Status: proposed  
Execution boundary: offline compilation and MuJoCo replay only

## 1. Architecture objectives

The v0.1 architecture must prove three claims:

1. Motion sources and robot targets can be decoupled through a stable intermediate representation.
2. Retargeting transformations can be inspected and tested pass by pass.
3. Unitree and AgiBot X2 SDK integrations can consume the same Motion IR contract without requiring physical hardware in CI.

Anything not required to prove those claims is deferred.

## 2. System context

```text
                 +---------------------+
BVH ------------>| BVH frontend        |
                 +----------+----------+
                            |
                            v
                 +---------------------+
                 | Canonical motion    |
                 +----------+----------+
                            |
URDF + profile ------------>|
                            v
                 +---------------------+
                 | Compiler pipeline   |
                 | - normalize         |
                 | - map               |
                 | - constrained IK    |
                 | - smooth            |
                 | - validate          |
                 +----------+----------+
                            |
                            v
                 +---------------------+
                 | Motion IR bundle    |
                 | + validation report |
                 +----------+----------+
                            |
                            v
                 +---------------------+
                 | MuJoCo replay       |
                 +---------------------+
```

## 3. Package boundaries

### `ohmc.frontends.bvh`

Responsibilities:

- Parse hierarchy, channels, frame rate, and samples.
- Reject malformed or ambiguous files.
- Convert units and axes into the documented canonical convention.
- Emit `CanonicalMotion` and provenance metadata.

Must not:

- Know robot joint names.
- Run inverse kinematics.
- Change motion timing without recording a pass.

### `ohmc.ir`

Responsibilities:

- Define immutable or copy-on-write data models.
- Validate schema versions.
- Serialize canonical motion, Motion IR, pass records, and reports.
- Calculate stable content hashes.

Must not:

- Import MuJoCo or ROS 2.
- Contain vendor-specific message types.
- Hide units or coordinate conventions.

### `ohmc.robot_profiles`

Responsibilities:

- Load a declarative robot profile.
- Verify referenced model hashes when configured.
- Resolve semantic links and the controllable-joint whitelist.
- Expose constraints to solvers and validators.

Must not:

- Infer hardware authority from URDF.
- Add every movable joint automatically.
- Bundle restricted robot assets.

### `ohmc.passes`

Responsibilities:

- Implement composable transformations.
- Declare required input fields and generated output fields.
- Record configuration, metrics, warnings, and hashes.
- Fail explicitly when preconditions are missing.

Initial passes:

- `normalize_frames`
- `resample_timeline`
- `scale_morphology`
- `map_semantics`
- `constrained_ik`
- `smooth_trajectory`
- `validate_trajectory`

### `ohmc.solvers`

Responsibilities:

- Provide solver-neutral problem definitions.
- Implement the initial constrained IK backend.
- Return structured status, residuals, iteration counts, and failure reasons.

Must not:

- Silently use the previous frame when a solve fails.
- Clamp results without reporting the original violation.
- Mark a partially solved frame as fully valid.

### `ohmc.validators`

Responsibilities:

- Validate joint order, timestamps, finite values, and schema invariants.
- Check position, velocity, acceleration, and configured collision constraints.
- Calculate contact drift and kinematic residuals.
- Produce JSON and human-readable reports.

### `ohmc.backends.mujoco`

Responsibilities:

- Load an explicitly supported model fixture.
- Map Motion IR joints to simulator joints.
- Replay at the artifact's declared timing.
- Record replay diagnostics and optional video output.

Must not:

- Modify the compiled trajectory during replay.
- Treat simulator playback as proof of hardware safety.

### `ohmc.vendors.unitree`

Responsibilities:

- Resolve pinned `unitree_sdk2`, `unitree_ros2`, and `unitree_mujoco` revisions.
- Translate Motion IR joint semantics into supported Unitree model interfaces.
- Provide G1/H1-family profiles backed by compatible official model assets.
- Build adapter conformance tests against simulation and recorded messages.
- Keep hardware transport disabled unless an explicit hardware profile and operator gate are selected.

### `ohmc.vendors.agibot_x2`

Responsibilities:

- Resolve or import the official AimDK artifact and validate its checksum/version.
- Load the official X2 URDF dependency and declared joint profile.
- Translate Motion IR into AimDK/ROS 2 interface objects.
- Test against X2 interface fixtures without requiring a live robot.
- Separate task-level AimDK operations from any future low-level motion-control backend.

The X2 adapter is part of the main build matrix. The SDK artifact may live in a local dependency cache rather than Git history when its redistribution license is incomplete.

### `ohmc.vendors`

Shared responsibilities:

- Parse `vendor/vendor-lock.yaml`.
- Fetch, import, cache, and checksum dependencies.
- Detect installed SDK versions.
- Enforce the compatibility matrix.
- Produce software-bill-of-materials and license reports.

### `ohmc.cli`

Initial commands:

```text
ohmc inspect-source INPUT.bvh
ohmc canonicalize-bvh INPUT.bvh --source-convention CONVENTION --source-length-unit UNIT --source-license SPDX --output canonical.json
ohmc inspect-robot PROFILE.yaml
ohmc simulate INPUT.bvh --target TARGET --source-license SPDX --source-convention CONVENTION --source-length-unit UNIT --output BUILD_DIR
ohmc validate-ir BUILD_DIR/motion.json
ohmc replay BUILD_DIR/motion.json --backend mujoco
ohmc report BUILD_DIR/report.json
ohmc vendor status
ohmc vendor sync unitree
ohmc vendor import agibot-x2 /path/to/official-aimdk.zip
ohmc vendor verify
```

The CLI is an orchestration layer. Core packages remain callable as libraries.

## 4. Data contracts

### 4.1 Canonical motion

Required fields:

- Schema version.
- Source hash and provenance.
- Strictly increasing timestamps in seconds.
- Canonical frame convention identifier.
- Skeleton hierarchy.
- Ordered joint hierarchy with parent indices and rest offsets in metres.
- Local rotations as normalized `x, y, z, w` quaternions.
- World joint positions and rotations for every sample.
- Explicit source coordinate convention and source-to-metre scale provenance.

The implemented v0.1 contract is documented in
[`CANONICAL_MOTION.md`](CANONICAL_MOTION.md). Missing-data masks remain planned
for video and incomplete mocap frontends.

### 4.2 Robot profile

Required fields:

- Profile schema version.
- Robot and model identifiers.
- Model path or resolver and optional expected hash.
- World and base frame conventions.
- Semantic-link mapping.
- Ordered controllable-joint whitelist.
- Neutral configuration.
- Joint constraints.
- End-effectors.
- Solver weights and tolerances.

### 4.3 Motion IR bundle

A build directory is self-describing:

```text
build/example/
  manifest.json
  canonical-motion.json
  motion.source.json
  motion.json
  replay-report.json
  interface-fixture.json
  configs/
    robot-profile.yaml
    semantic-mapping.yaml
```

The v0.2 prototype currently emits `manifest.json`, source/mapped Motion IR,
replay report, interface fixture, and copied profile/mapping configuration.
Dedicated provenance and pass-log files remain planned; provenance hashes and
pass records currently live in the manifest and Motion IR respectively.

Large numeric arrays may move to a binary container after profiling. The manifest and schema remain readable and versioned.

### 4.4 Vendor lock

The vendor lock records:

- Official source or download location.
- Exact Git commit, release, or artifact version.
- SHA-256 checksum where applicable.
- Declared license and location of the license evidence.
- Acquisition mode: `git`, `official_download`, or `system`.
- Supported operating systems, architectures, ROS 2 distributions, and robot models.
- Whether redistribution inside release archives is enabled.

## 5. Coordinate conventions

v0.1 must choose one convention and enforce it at boundaries. Proposed canonical convention:

- Right-handed coordinates.
- `+Z` is up.
- `+X` is forward.
- `+Y` is left.
- Linear units are metres.
- Angular units are radians.
- Quaternions use an explicitly named component order in the schema.
- Timestamps are seconds from the first frame and strictly monotonic.

Every frontend converts into this convention. Every backend converts out of it. Tests include axis-labelled poses and rotations so that a visually plausible but mirrored result cannot pass unnoticed.

## 6. Compilation pipeline

### Stage 0: preflight

- Validate all schemas.
- Hash source, model, profile, and configuration.
- Confirm licenses for bundled test fixtures.
- Resolve output directory without overwriting an existing build unless explicitly requested.
- Resolve selected vendor dependencies and verify their versions, licenses, and checksums.

### Stage 1: ingestion

- Parse BVH.
- Create canonical skeleton motion.
- Report unsupported channels, missing landmarks, and source anomalies.

### Stage 2: normalization

- Convert axes and units.
- Normalize timestamps.
- Preserve original duration.
- Produce reference-pose diagnostics.

### Stage 3: semantic mapping

- Resolve human landmarks to robot links.
- Calculate morphology scales.
- Reject missing required semantics.
- Record intentionally unmapped source and robot joints.

### Stage 4: constrained IK

For each frame, solve an objective composed of:

- End-effector position and orientation tracking.
- Torso and root tracking.
- Neutral-pose regularization.
- Temporal regularization against adjacent frames.
- Joint-limit constraints.
- Pinned-contact constraints when contact is declared.

The exact optimizer is replaceable. Its convergence criteria and fallback policy are configuration, not hidden constants.

### Stage 5: smoothing

- Operate on the complete trajectory.
- Respect position limits and contact intervals.
- Calculate pre/post velocity, acceleration, and jerk statistics.
- Reject changes that degrade configured contact thresholds.

### Stage 6: validation

- Run schema and numerical checks.
- Calculate benchmark metrics.
- Assign `pass`, `warning`, or `fail` with machine-readable reasons.
- Never convert `fail` to `warning` merely to permit replay.

### Stage 7: packaging

- Write the build directory atomically.
- Include resolved configuration and provenance.
- Generate a concise terminal summary.

## 7. Failure model

All failures have a stable category:

- `INPUT_INVALID`
- `SCHEMA_UNSUPPORTED`
- `MODEL_MISMATCH`
- `SEMANTIC_MAPPING_INCOMPLETE`
- `SOLVER_INFEASIBLE`
- `SOLVER_DID_NOT_CONVERGE`
- `CONSTRAINT_VIOLATION`
- `COLLISION_DETECTED`
- `BACKEND_INCOMPATIBLE`
- `REPLAY_FAILED`

An error record includes stage, frame range when applicable, human-readable explanation, relevant metrics, and suggested investigation. Suggestions must not conceal the failed condition.

## 8. Configuration precedence

Configuration resolves in this order:

1. Versioned project defaults.
2. Robot profile defaults.
3. User compile configuration.
4. Explicit CLI flags.

The fully resolved configuration is stored in the output bundle. Environment variables are limited to operational concerns such as cache location; they must not silently change solver semantics.

## 9. Determinism policy

- Dependencies are pinned for reference builds.
- Randomized algorithms require an explicit stored seed.
- Parallel solvers must document expected numerical variation.
- Golden tests compare structural equality and numerical tolerances separately.
- Generated videos are not canonical artifacts and are excluded from deterministic comparisons.

## 10. Test strategy

### Unit tests

- BVH channel parsing and hierarchy validation.
- Axis and unit conversion.
- Quaternion ordering and normalization.
- Schema round trips.
- Robot whitelist resolution.
- Limit and derivative calculations.
- Pass provenance and hashing.

### Property tests

- Resampling preserves duration.
- Serialization round trips preserve semantics.
- Joint ordering never changes silently.
- Smoothing does not create configured limit violations.
- Coordinate conversion and inverse conversion agree within tolerance.

### Integration tests

- Licensed BVH fixture to Motion IR.
- Motion IR schema validation.
- MuJoCo model loading and joint mapping.
- Headless replay in CI.
- Full report generation for passing and intentionally failing fixtures.
- Unitree adapter build and simulated-message translation.
- AgiBot X2 AimDK package detection and interface translation.
- Vendor checksum mismatch and unsupported-version failures.

### Regression tests

- Golden metrics for selected motions.
- Known mirrored-axis failure.
- Known unreachable-pose failure.
- Known foot-sliding case.
- Known joint-order mismatch.

## 11. Technology choices

Proposed for v0.1:

- Python 3.10+ for pipeline orchestration and early solver iteration.
- Pydantic or equivalent typed schema validation.
- NumPy/SciPy for numerical infrastructure.
- Pinocchio, Pink, or another reviewed kinematics layer selected through a prototype comparison.
- MuJoCo for headless and visual replay.
- `pytest` for tests.
- JSON Schema for public artifact validation.
- ROS 2 Humble integration only after the offline core is stable.
- Pinned Unitree and AgiBot X2 adapter environments in the primary CI matrix.

The solver dependency is intentionally undecided in this architecture document. Selection requires a small benchmark covering licensing, URDF support, constraints, convergence diagnostics, installation complexity, and deterministic behavior.

## 12. Security, privacy, and licensing checks

CI should reject:

- Common credential formats and private keys.
- Absolute developer-machine paths in committed fixtures.
- Unapproved large binaries.
- Files without known provenance in licensed fixture directories.
- Dependency licenses outside the approved policy.

Before public release, repository history must also be scanned. Removing a secret or restricted asset from the latest tree does not remove it from Git history.

## 13. v0.1 acceptance test

The reference acceptance run is:

```text
Given:
  one redistributable BVH fixture,
  one redistributable humanoid model,
  one versioned robot profile,
  and one pinned compile configuration,

When:
  a clean Linux environment runs the documented compile,
  validate, and headless replay commands,

Then:
  the build bundle validates against the public schema,
  all pass records and hashes are present,
  no configured hard constraint is violated,
  replay completes without modifying the artifact,
  and a repeated run matches the documented tolerances.
```

Physical-robot behavior is not part of this acceptance test.

## 14. Deferred decisions

- Final solver library.
- Binary storage format for long trajectories.
- Contact inference algorithm.
- Collision library and geometry simplification policy.
- ROS 2 custom message necessity.
- Web visualization technology.
- Exact hardware-enablement gate for each vendor adapter.

Each deferred decision should receive a short architecture decision record before implementation becomes difficult to reverse.
