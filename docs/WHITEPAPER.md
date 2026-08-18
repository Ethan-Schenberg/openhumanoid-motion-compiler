# OpenHumanoid Motion Compiler: Project White Paper

Status: design proposal  
Target milestone: v0.1, offline simulation prototype

## Abstract

Humanoid-motion retargeting is commonly delivered as a robot-specific demonstration: a particular motion source is converted through an undocumented set of scripts and played on a single platform. The resulting trajectory may look convincing, but its coordinate conventions, contact assumptions, optimization objectives, and safety constraints are difficult to audit or reuse.

OpenHumanoid Motion Compiler (OHMC) proposes a different abstraction. Human motion is treated as source code; a canonical motion representation and Motion IR act as intermediate representations; constraint solvers and validators act as compiler passes; simulators and robot integrations act as backends. This structure makes transformations explicit, enables repeatable validation, and allows new robots or motion sources to be integrated without rewriting the entire pipeline.

The first release is strictly offline. It compiles BVH motion for a redistributable humanoid model, validates the output, and replays it in MuJoCo. Physical-robot execution remains outside the trusted boundary until separate adapter, licensing, and safety milestones are satisfied.

## 1. Problem statement

A reusable humanoid retargeting system must reconcile several incompatible spaces:

- Human and robot bodies have different proportions and degrees of freedom.
- Source formats use different axes, units, frame hierarchies, and sampling rates.
- A visually similar pose may be dynamically invalid for a robot.
- Foot contacts can be lost during naive inverse kinematics, producing sliding or unstable motion.
- URDF geometry is useful model evidence but does not, by itself, describe a live controller contract.
- Simulator command interfaces and hardware command interfaces have different timing and safety requirements.
- Vendor-specific packages may not be redistributable.

These are not isolated conversion bugs. They are interface and provenance problems. OHMC therefore makes each assumption a versioned, inspectable part of the compilation artifact.

## 2. Project goals

OHMC aims to provide:

1. A versioned, robot-independent Motion IR.
2. Reusable frontends for human-motion formats.
3. Declarative robot profiles describing semantic mappings and constraints.
4. Composable passes for retargeting, smoothing, contact handling, and validation.
5. Simulator backends with reproducible replay.
6. Standard ROS 2 integration points that do not depend on a single robot vendor.
7. Maintained, first-class integrations for Unitree and AgiBot X2 official SDK ecosystems.
8. Benchmarks that report quality and constraint violations rather than relying only on demonstration videos.

## 3. Non-goals

OHMC is not:

- A claim that any generated trajectory is safe for physical execution.
- A replacement for a robot manufacturer's controller, firmware, or safety system.
- A universal dynamics solver in its first release.
- A mirror for vendor artifacts whose licenses do not permit redistribution.
- A one-click video-to-robot product in v0.1.
- A benchmark whose only metric is visual similarity.

## 4. Compiler model

### 4.1 Frontends

Frontends parse motion sources and normalize them into a canonical skeleton sequence. Each frontend must declare:

- Input format and version.
- Units and axis conventions.
- Source frame rate and timestamps.
- Skeleton hierarchy.
- Confidence or missing-data semantics, when applicable.
- Source license and provenance metadata.

The v0.1 frontend supports BVH. Video pose estimation and live motion capture are later extensions.

### 4.2 Canonical skeleton representation

The canonical representation isolates human-source conventions from robot conventions. It contains named semantic landmarks such as pelvis, torso, head, shoulders, elbows, wrists, hips, knees, ankles, heels, and toes.

It records both local and world transforms where available. Coordinate conversion happens once at ingestion and is tested with known poses, rather than being repeated implicitly by downstream modules.

### 4.3 Motion IR

Motion IR is the stable contract between retargeting passes and output backends. It is not a stream of unnamed joint arrays. At minimum, an artifact contains:

- Schema version.
- Source provenance and content hashes.
- Robot model identifier and model hash.
- World, base, and joint-frame conventions.
- Ordered semantic joint names.
- Monotonic timestamps.
- Position targets and optional velocity/acceleration targets.
- End-effector poses and contact phases.
- Per-frame solver status and residuals.
- Applied pass history and configuration hashes.
- Constraint limits used during compilation.
- Validation summary and unresolved warnings.

An illustrative document shape is:

```yaml
schema: ohmc.motion_ir/v0.1
source:
  kind: bvh
  sha256: "..."
robot:
  profile: generic_humanoid_v1
  model_sha256: "..."
frames:
  world: z_up_right_handed
  base: pelvis
trajectory:
  rate_hz: 50
  joints: [left_hip_pitch, left_knee_pitch]
  samples: []
contacts:
  - effector: left_foot
    intervals: []
passes:
  - name: constrained_ik
    config_sha256: "..."
validation:
  status: warning
  report: reports/example.json
```

The final schema will use a machine-validated representation; the YAML above communicates semantics only.

### 4.4 Compiler passes

Passes consume and produce explicit artifacts. Proposed passes include:

1. **Frame normalization** — converts axes, handedness, units, and root orientation.
2. **Temporal resampling** — produces a monotonic target rate without silently changing duration.
3. **Morphology scaling** — adapts human limb proportions to the target robot.
4. **Semantic mapping** — connects canonical landmarks to robot links and controllable joints.
5. **Contact inference/import** — identifies candidate stance intervals and records confidence.
6. **Constrained IK** — minimizes pose error subject to joint and contact constraints.
7. **Trajectory smoothing** — reduces jerk without violating pinned contacts or joint limits.
8. **Validation** — evaluates limits, residuals, discontinuities, collision distance, and contact drift.

Every pass records its name, version, parameters, input hash, output hash, duration, and diagnostics.

### 4.5 Backends

Backends translate validated Motion IR into a target environment. A backend may reject an artifact even when the core validator accepts it.

Planned backend classes:

- MuJoCo replay backend.
- ROS 2 message and rosbag export backend.
- `ros2_control` reference-interface backend.
- Unitree backend based on pinned `unitree_sdk2`, `unitree_ros2`, and `unitree_mujoco` revisions.
- AgiBot X2 backend based on AimDK, X2 ROS 2 interfaces, and the X2 robot model.

Physical backends are intentionally absent from v0.1.

Vendor support is not an afterthought. SDK discovery, version compatibility, model profiles, message translation, simulation, and adapter conformance tests are maintained in the primary repository. The generic Motion IR core remains vendor-neutral so the two integrations exercise the same contract rather than becoming separate pipelines.

## 5. Robot profiles

A robot profile is a declarative package, not a hidden Python dictionary. It defines:

- Model reference and expected hash.
- Semantic link mapping.
- Controllable-joint whitelist and stable ordering.
- Neutral pose.
- Joint position, velocity, and acceleration constraints.
- End-effectors and contact geometry.
- Self-collision exclusions with justification.
- Solver weights and tolerances.
- Backend compatibility.
- Profile license and provenance.

The controllable-joint whitelist is independent from the set of non-fixed joints found in URDF. This prevents a model artifact from being mistaken for authority to command every modeled joint.

## 6. Validation and benchmarks

OHMC reports several dimensions rather than a single success flag.

### 6.1 Kinematic fidelity

- Mean and percentile end-effector position error.
- Mean and percentile orientation error.
- Root and torso tracking error.
- Joint-space discontinuity counts.

### 6.2 Constraint compliance

- Joint-position limit violations.
- Velocity and acceleration limit violations.
- Minimum self-collision distance.
- Foot-contact drift during stance intervals.
- Solver failure or fallback rate.

### 6.3 Reproducibility

- Input and model hashes.
- Configuration and dependency lock hashes.
- Deterministic output comparison within documented numerical tolerances.
- Replay success in the supported simulator version.

### 6.4 Performance

- Compilation time per motion second.
- Peak memory usage.
- Solver iterations per frame.
- Percentage of frames requiring fallback behavior.

The benchmark suite must preserve failed examples. Removing difficult motions from published results would undermine the project's auditability.

## 7. Safety architecture

OHMC uses a staged trust model:

```text
Untrusted source motion
        |
        v
Canonical parsing and provenance checks
        |
        v
Kinematic compilation
        |
        v
Static validation
        |
        v
Simulator replay
        |
        v
Hardware review boundary (outside v0.1)
```

A green offline report means only that documented offline checks passed. It does not certify balance, actuator feasibility, communications timing, controller compatibility, or physical safety.

Any future hardware adapter must add its own gates, including controller-state verification, command ownership, emergency-stop readiness, rate and limit enforcement, operator confirmation, and post-action state inspection. A timed-out physical action must never be automatically retried.

## 8. Vendor SDK delivery and licensing

The intended license for original OHMC source code is Apache-2.0, subject to confirmation before the first public release. Each dependency and example asset must be reviewed independently.

OHMC uses three dependency-delivery modes:

1. **Pinned source dependency** — for official repositories with a redistribution-compatible license. The exact commit and license are recorded in `vendor/vendor-lock.yaml`.
2. **Verified official download** — for an SDK distributed through an official download channel whose package does not provide a complete redistribution grant. `ohmc vendor sync` downloads or imports the official artifact, checks its hash, and installs it into a local cache outside Git history.
3. **System dependency** — for SDKs already installed on a robot or development image. OHMC discovers the version and validates it against the compatibility matrix.

For the initial vendor set:

- Unitree `unitree_sdk2`, `unitree_ros2`, and `unitree_mujoco` are official BSD-3-Clause repositories and are pinned as source dependencies.
- The AgiBot X2 URDF package declares MIT in `package.xml` and is versioned as an official model dependency with its source URL and checksum.
- The inspected AimDK v1.0.0 artifact is an official SDK download, but its ROS package manifests contain `TODO: License declaration` rather than a complete project license. OHMC therefore includes the AimDK adapter, resolver, checksum, and compatibility tests while obtaining the SDK from the official channel instead of copying the 157 MB artifact into public Git history.

The public repository must not contain:

- Vendor SDK source or binaries when the applicable license does not grant redistribution.
- Robot models, meshes, motions, datasets, or documentation without compatible licenses.
- Credentials, SSH configuration, private network information, or device identifiers.
- Field logs or recordings that expose people, facilities, or proprietary systems.
- Generated artifacts whose source license prohibits redistribution.

Users should not have to search for dependencies manually. The vendor resolver owns acquisition, version detection, checksum validation, caching, and clear remediation. A dependency remaining outside Git history does not make its platform a second-class integration.

## 9. ROS 2 integration strategy

The core compiler remains usable without a running ROS graph. ROS 2 packages provide integration rather than owning the mathematical core.

Proposed ROS 2 interfaces include:

- A compile action for long-running offline jobs.
- Diagnostic messages for pass progress and validation results.
- A trajectory preview topic using standard message types where semantics match.
- rosbag export for repeatable inspection.
- A future `ros2_control` chainable controller or reference-interface bridge.

Standard interfaces are preferred over project-specific messages. Custom messages are introduced only when Motion IR semantics cannot be represented without losing provenance or validation data.

## 10. Governance and maintenance

The project should earn trust through maintenance behavior, not repository presentation.

Minimum governance artifacts before a public v0.1 release:

- Contribution guide.
- Code of conduct.
- Security policy.
- Supported-version matrix.
- Decision records for schema and safety changes.
- Issue and pull-request templates.
- Reproducible development environment.
- Automated formatting, tests, schema validation, and license checks.

Motion IR changes follow semantic versioning. Breaking schema changes require a migration note and, where practical, a converter.

## 11. Milestones

### M0: Specification

- Publish Motion IR draft.
- Select redistributable BVH and humanoid model fixtures.
- Define coordinate and contact conventions.
- Create deterministic schema tests.
- Publish the vendor lock, compatibility matrix, and SDK resolver contract.

### M0.5: Vendor foundation

- Pin Unitree SDK2, ROS 2, and MuJoCo source revisions.
- Add Unitree G1/H1-family robot profiles selected from licensed official assets.
- Add the AgiBot X2 official URDF dependency and profile.
- Implement AimDK local-package import and checksum verification.
- Build adapter conformance tests against recorded or simulated interfaces.

### M1: Offline compiler v0.1

- Parse BVH.
- Load and hash a robot model.
- Execute semantic mapping and constrained IK.
- Emit Motion IR plus validation report.
- Replay the artifact in MuJoCo.
- Compose import, mapping, replay, provenance, and non-executable adapter
  fixtures through a one-command simulation evidence bundle.

### M2: Benchmark and visualization

- Add benchmark motions and regression thresholds.
- Show source and robot motion side by side.
- Export machine-readable result summaries.

### M3: ROS 2 integration

- Add ROS 2 compile action and diagnostic interfaces.
- Export standard trajectories and rosbag fixtures.
- Demonstrate with simulation-only controllers.
- Compile the Unitree and AgiBot X2 adapters in CI-compatible dependency environments.

### M4: Hardware-adapter incubation

- Define an adapter safety contract.
- Promote the already integrated Unitree and AgiBot X2 adapters from simulation/recorded-interface mode to hardware-gated mode.
- Keep hardware execution opt-in and outside the default command path.
- Require platform-specific validation before any real-robot demonstration.

## 12. v0.1 success criteria

v0.1 is complete only when a fresh environment can:

1. Install pinned dependencies from documented instructions.
2. Compile the included licensed BVH fixture for the included licensed robot fixture.
3. Validate the generated Motion IR against the published schema.
4. Produce a machine-readable report with no undisclosed constraint violations.
5. Replay the trajectory in MuJoCo without manual file editing.
6. Re-run the pipeline and reproduce outputs within published numerical tolerances.
7. Execute all tests without access to a physical robot or proprietary SDK.

## 13. Long-term vision

OHMC can become shared infrastructure for comparing retargeting methods, robot morphologies, and motion-controller backends. Its strongest contribution would not be a spectacular single demonstration, but a common, inspectable contract connecting motion research to ROS 2 engineering while preserving the distinction between visual plausibility, simulated feasibility, and verified hardware behavior.
