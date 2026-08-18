# Vendor SDK Integration Policy

Status: initial vendor baseline for OHMC v0.1

## Principle

Unitree and AgiBot X2 are first-class OHMC platforms. Their adapters, dependency resolvers, compatibility tests, robot profiles, and documentation belong in the main repository.

"Included" does not require every upstream binary to be committed into the same Git object database. A dependency is considered included when OHMC owns a pinned, tested, one-command path that obtains or imports the official artifact, verifies it, and exposes a stable adapter contract.

## Initial matrix

| Vendor | Component | Upstream status | Acquisition | Redistribution in OHMC releases |
|---|---|---|---|---|
| Unitree | `unitree_sdk2` | Official public repository, BSD-3-Clause | Pinned Git source | Allowed with license and notice preservation |
| Unitree | `unitree_ros2` | Official public repository, BSD-3-Clause | Pinned Git source | Allowed with license and notice preservation |
| Unitree | `unitree_mujoco` | Official public repository, BSD-3-Clause | Pinned Git source | Allowed with license and notice preservation |
| AgiBot X2 | X2 URDF v1.3.0 | Official download; ROS package declares MIT | Verified official download | Candidate for inclusion after adding complete license/notice evidence |
| AgiBot X2 | AimDK v1.0.0 artifact | Official download; inspected package manifests say `TODO: License declaration` | Official download or local import, SHA-256 verified | Disabled until an explicit redistribution license is located or obtained |

## Integration levels

Every vendor backend advances through the same levels:

1. **L0 — dependency verified:** source/version/license/checksum recorded.
2. **L1 — model loaded:** robot model and semantic joint profile validate.
3. **L2 — adapter compiled:** interface translation builds without hardware.
4. **L3 — simulation passed:** Motion IR replays through the vendor simulator or model backend.
5. **L4 — recorded-interface passed:** telemetry and command encoding pass against fixtures.
6. **L5 — hardware gated:** physical execution exists behind platform-specific checks and explicit operator enablement.

v0.1 targets L3 for Unitree and at least L2 for AgiBot X2. The X2 target rises to L3 when the official model and simulation path are reproducibly packaged.

Current implemented status (2026-08-18): both vendor profiles have reached L1.
Their schemas, model hashes, joint orders, whitelists, limits, exclusions, and
offline semantic mappings are tested in CI. Both platforms now also have
schema-validated interface-order fixtures: Unitree G1 `LowCmd` and AgiBot X2
`JointCommandArray`. These fixtures are pre-L2 contract evidence, not a claim
that either SDK adapter has compiled. SDK message adapters have not yet reached
L2, and no hardware transport is present.

The exact fixture format and its non-execution boundary are documented in
[`INTERFACE_FIXTURES.md`](INTERFACE_FIXTURES.md).

## Unitree integration

The first implementation should support one humanoid reference model while keeping the SDK layer reusable across supported Unitree robots.

Required components:

- `unitree_sdk2` for the official DDS SDK.
- `unitree_ros2` for ROS 2 message and communication integration.
- `unitree_mujoco` for official MuJoCo-side testing.
- Official robot description selected for the first reference model.

OHMC must preserve upstream BSD-3-Clause notices. Local patches should be small, documented, and submitted upstream where generally useful.

## AgiBot X2 integration

The X2 backend is built around:

- AimDK ROS 2 messages and task-level interfaces.
- X2 semantic joint mapping and explicit controllable-joint whitelist.
- Official X2 URDF for model and frame conversion.
- MuJoCo/offline checks before any physical execution layer.

The resolver supports:

```text
ohmc vendor import agibot-x2 /path/to/aimdk-aarch64-artifacts.zip
ohmc vendor verify agibot-x2
ohmc vendor doctor agibot-x2
```

Import copies or links the artifact into an ignored local cache, verifies its SHA-256, inspects its ROS packages, and reports compatibility. It does not rely on an undocumented file already existing on a robot.

## Updating a dependency

A vendor update requires:

1. Update the lock entry.
2. Record upstream changelog and license differences.
3. Run adapter compile tests.
4. Run Motion IR mapping and simulation regression tests.
5. Compare joint names, ordering, limits, coordinate conventions, and message definitions.
6. Publish a compatibility note.

Vendor `latest` branches are never consumed silently by a release build.

## Current evidence snapshot

This design snapshot recorded the following exact inputs on 2026-08-16:

- Unitree `unitree_sdk2` HEAD: `21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b`
- Unitree `unitree_ros2` HEAD: `668d1ec5a05d1c38d3306bdca7d59f2ba3581a88`
- Unitree `unitree_mujoco` HEAD: `ae6a8403e272733e9996ef59990880330496177f`
- AgiBot AimDK local official artifact SHA-256: `5bbcf724d54fb28f153db0d272f9acb7906bb1d2cac7dd7ccdc699a5c7eeab35`
- AgiBot X2 URDF v1.3.0 SHA-256: `e3e14a9631054a14659a2fb9445c4cec8224d88bd489b071bc9ea97853918bf0`

These commits and checksums are inputs to the first prototype, not permanent claims about the newest upstream versions.
