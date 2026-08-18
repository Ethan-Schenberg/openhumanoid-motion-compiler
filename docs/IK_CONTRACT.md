# Solver-neutral constrained IK v0.1

OHMC separates an inverse-kinematics problem from the solver that executes it.
The JSON contracts make target generation, numerical solving, residuals, and
failure states independently inspectable.

## Artifacts

- `ohmc.ik_task_map/v0.1` declares profile semantics used as variables,
  canonical source joints, robot target frames, weights, tolerances, and every
  solver parameter.
- `ohmc.ik_problem/v0.1` resolves those semantics to exact robot joint names and
  limits, pins canonical/profile/solver-model hashes, anchors source and target
  reference poses, and emits every frame's Cartesian targets.
- `ohmc.ik_result/v0.1` records every frame as `solved` or `failed`, its joint
  vector, iteration count, per-task and peak residuals, and active joint limits.

All three contracts are independent of hardware transport. The problem and
result explicitly record `hardware_commands_sent: false`.

## Reference-delta tasks

The v0.1 task kind is `frame_position`. At the first canonical sample, the
source landmark and robot target frame are each anchored in their own model.
Later targets apply the scaled canonical displacement to the robot anchor:

```text
target(t) = robot_reference
          + scale * (canonical(t) - canonical_reference)
```

This preserves motion displacement without pretending that a human joint and a
robot link have identical absolute world coordinates or limb proportions.

## Deterministic reference solver

The bundled MuJoCo reference solver uses bounded damped least squares. Its
damping, maximum iteration count, per-iteration angular step, neutral-pose
weight, and previous-frame temporal weight all come from the task map. Each
iteration uses MuJoCo's analytical body-position Jacobian and clips the proposed
joint vector to the profile limits.

There is no hidden clamp-success path. A frame is solved only when every task is
within its declared tolerance. Otherwise it is `failed`; a result containing
any failed frame has overall status `fail` and cannot be converted to Motion IR.

## Current coverage boundary

The small official-model smoke maps track one canonical left-knee landmark with
three left-hip variables. The larger multi-limb benchmark consumes all 16
canonical full-body landmarks as Cartesian position tasks while using all 29
G1 variables and all 30 available X2 variables. On the included three-frame
CC0 motion, both official models solve every frame below the declared 5 mm task
tolerance. A `full_body_landmarks_v1` target now fails before packaging if
source or task-map landmark coverage is incomplete.

The G1 29DoF model does not expose an articulated head frame. Its head landmark
therefore uses an explicitly named `head_position_proxy` task anchored to the
torso; it must not be interpreted as head-orientation tracking. The manifests
report `full_body_landmark_position_tasks: true` but remain deliberately marked
as `constrained_partial_body_ik: true` and `constrained_whole_body_ik: false`.

The 16-landmark maps are versioned as `full_body_unitree_g1_v2` and
`full_body_agibot_x2_v2`. The earlier nine-task `v1` maps remain in the
repository so historical evidence manifests retain a resolvable configuration
identity; active multi-limb simulation targets select `v2`.

Whole-body status still requires frame-orientation tasks, contact constraints,
balance metrics, and collision checks in addition to the implemented bilateral
legs, waist, arms, available head variables, and 16/16 landmark-position
coverage. Adding position-task names without those acceptance conditions does
not change that status.

The reference solver checks the finiteness of the stacked Jacobian, error
vector, normal equations, and solved update. A non-finite value or singular
normal equation raises an explicit compilation error rather than leaking a
numerically suspect trajectory into Motion IR.
