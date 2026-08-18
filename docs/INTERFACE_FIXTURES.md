# Offline Vendor Interface Fixtures

Status: implemented contract-conformance layer for OHMC v0.1

## Purpose

OHMC can translate robot-profile-mapped Motion IR into reviewable JSON fixtures
that mirror the ordering and fields of two official vendor command interfaces:

- Unitree G1 29DoF: `unitree_hg.msg.dds_.LowCmd_`
- AgiBot X2 Ultra: `aimdk_msgs/msg/JointCommandArray`

These files answer a narrow engineering question: does OHMC put each mapped
joint target into the correct vendor-defined slot and retain every required
field? They are not wire messages and cannot be published by OHMC.

## Safety properties

Every fixture is schema constrained to contain:

- `purpose: interface_order_conformance_only`
- `transport: disabled`
- `executable: false`
- the selected robot profile and model SHA-256
- a stable SHA-256 of the source Motion IR

OHMC never fills an absent target with zero. Missing positions stay `null` and
are marked `present: false`. Controller modes, gains, efforts, damping, and
message headers stay `null`. The command has no option that enables DDS, ROS 2,
or robot transport.

## Unitree G1 contract

The Unitree fixture contains exactly 29 `motor_cmd` slots in the `JointIndex`
order declared by the pinned `unitree_sdk2` G1 header. Each slot retains the
official low-level fields `mode`, `q`, `dq`, `kp`, `kd`, and `tau`; only `q` may
contain a mapped offline target. Top-level `mode_pr` and `mode_machine` are
unset. The recorded topic name is `rt/lowcmd`, but no publisher exists.

## AgiBot X2 contract

The AgiBot fixture mirrors the pinned AimDK definitions:

```text
JointCommandArray = MessageHeader header + JointCommand[] joints
JointCommand = name + position + velocity + effort + stiffness + damping
```

Each frame records four groups using the documented topics and order:

| Group | Topic | Slots |
|---|---|---:|
| leg | `/aima/hal/joint/leg/command` | 12 |
| waist | `/aima/hal/joint/waist/command` | 3 |
| arm | `/aima/hal/joint/arm/command` | 14 |
| head | `/aima/hal/joint/head/command` | 2 |

The second head slot is `head_pitch_joint`. AimDK reserves it but marks it
unavailable, so the fixture explicitly sets `available: false`, `present:
false`, and all command values to `null`.

## Usage

First map Motion IR to the selected robot profile, then encode the fixture:

```bash
ohmc encode-fixture build/unitree_g1_motion.json \
  --robot profiles/unitree_g1_29dof.yaml \
  --adapter unitree-g1-lowcmd \
  --output build/unitree_g1_lowcmd_fixture.json

ohmc encode-fixture build/agibot_x2_motion.json \
  --robot profiles/agibot_x2_ultra_aimdk_v1.yaml \
  --adapter agibot-x2-joint-command-array \
  --output build/agibot_x2_joint_command_fixture.json
```

The encoder rejects a mismatched profile, mismatched model hash, invalid Motion
IR, enabled hardware transport, and accidental output overwrite. The JSON Schema
is `schemas/vendor-interface-fixture-v0.1.schema.json`.

## Evidence and limitations

The field and slot contracts come from the exact dependencies pinned in
`vendor/vendor-lock.yaml`. CI tests the generated structure without downloading
or redistributing the vendor SDK artifacts.

This work does not satisfy the project's L2 "adapter compiled" milestone. That
requires compiling translations against the pinned SDK/message packages in a
licensed build environment. It also does not establish controller semantics,
safe gains, timing behavior, checksums, state transitions, simulation
stability, or physical-robot readiness.
