# Offline Robot Profiles

Status: model-level profiles for deterministic tests; hardware transport disabled

OHMC robot profiles turn vendor model evidence into an explicit, reviewable
contract. A profile fixes the model hash, controllable-joint whitelist, semantic
names, joint order, limits, exclusions, and body groups. Loading a profile never
connects to ROS 2, DDS, AimDK, or a physical robot.

## Support level

| Profile | Evidence | Whitelist | Current level |
|---|---|---:|---|
| Unitree G1 29DoF | Official `unitree_mujoco` MJCF at pinned commit `ae6a8403...` | 29 | L1 model/profile validation |
| AgiBot X2 Ultra | Official X2 URDF v1.3.0 plus AimDK 1.0 joint-order documentation | 30 | L1 model/profile validation |

L1 does not mean that an SDK adapter is compiled or that a robot can execute the
trajectory. Unitree SDK2/DDS and AgiBot AimDK/ROS 2 transport remain disabled.

## Unitree G1 29DoF

`profiles/unitree_g1_29dof.yaml` is derived from the official G1 29DoF MJCF in
`unitreerobotics/unitree_mujoco`. The profile records the exact upstream commit,
model SHA-256, joint names, order, and limits used by the offline contract.

Evidence:

- https://github.com/unitreerobotics/unitree_mujoco
- https://github.com/unitreerobotics/unitree_mujoco/blob/ae6a8403e272733e9996ef59990880330496177f/unitree_robots/g1/g1_29dof.xml

The official simulator integrates Unitree SDK2 and MuJoCo, but OHMC does not yet
claim SDK message compatibility merely because the model profile validates.

## AgiBot X2 Ultra

`profiles/agibot_x2_ultra_aimdk_v1.yaml` is derived from the verified
`X2_URDF-v1.3.0.zip` artifact and the public AimDK 1.0 joint contract. The URDF
contains 31 non-fixed revolute joints. The OHMC whitelist contains 30:

- 12 leg joints
- 3 waist joints
- 14 arm joints
- `head_yaw_joint`

`head_pitch_joint` is explicitly excluded. AimDK 1.0 reserves a two-element head
array but documents pitch as currently unavailable; head yaw also requires
supported head hardware. A future live profile must re-check the exact robot,
AimDK, firmware, and head configuration rather than inheriting model authority.

Evidence:

- https://x2-aimdk.agibot.com/en/latest/Interface/control_mod/joint_control.html
- https://x2-aimdk.agibot.com/zh-cn/latest/_downloads/2ffc9785259556f409e385974a7a0461/X2_URDF-v1.3.0.zip

The profile records both the archive SHA-256 and the SHA-256 of
`X2_URDF-v1.3.0/x2_ultra.urdf`. The archive is not redistributed by OHMC.

## Semantic mapping

The prototype semantic mapper resolves source channels into profile semantics,
then emits joints in the robot profile's declared order. It applies scale and
offset rules and rejects every mapped position outside the profile limits.

The included mapping is intentionally small:

```text
Hips.z_rotation     -> waist_yaw
LeftKnee.z_rotation -> left_knee (scale -1)
```

This proves deterministic profile resolution, ordering, sign conversion, and
limit enforcement. It is not whole-body retargeting. Unmapped source joints are
reported, and every output retains warnings that IK and hardware transport have
not been applied.
