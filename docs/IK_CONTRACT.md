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
three left-hip variables. The larger multi-limb benchmark maps track chest,
bilateral knees and ankles, and bilateral elbows and wrists: nine Cartesian
tasks using all 29 G1 variables and all 30 available X2 variables. On the
included three-frame CC0 motion, both official models solve every frame below
the declared 5 mm task tolerance.

The landmark report still shows task coverage as 9/16. Therefore both tiers are
deliberately reported as `constrained_partial_body_ik: true` and
`constrained_whole_body_ik: false`, even though the output joint vectors cover
the complete vendor control order.

Whole-body status requires bilateral legs, waist, arms, available head joints,
orientation tasks, contact constraints, and collision checks. Adding animation
or more joints without those acceptance conditions does not change that status.
