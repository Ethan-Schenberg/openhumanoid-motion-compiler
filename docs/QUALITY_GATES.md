# Trajectory Quality Gates v0.1

OHMC separates three questions that are often collapsed into “the motion
works”:

1. Are positions inside declared joint limits?
2. Are velocity and acceleration values within configured dynamic limits?
3. Does the trajectory cover the robot's complete controllable-joint contract?

The `ohmc.trajectory_quality/v0.1` report answers each question independently.
It never treats a missing limit as a passing limit.

## Derivative generation

`ohmc derive-kinematics` adds `velocity_targets` and
`acceleration_targets` to Motion IR. It uses three-point Lagrange finite
differences that support non-uniform timestamps:

- centered quadratic stencils for interior samples;
- one-sided quadratic stencils for endpoints;
- a secant velocity and zero acceleration for two samples;
- zero derivatives for a single sample.

The method exactly differentiates quadratic position data at irregular sample
times. Existing derivative vectors are not overwritten implicitly.

## Limit validation

For every mapped joint, the report records:

- minimum and maximum position;
- minimum distance to either position limit;
- maximum absolute velocity and configured velocity limit;
- maximum absolute acceleration and configured acceleration limit;
- `pass`, `fail`, or `not_configured` for each dynamic limit.

Any measured violation makes the report `fail` and prevents the one-command
simulation bundle from being published. A missing dynamic limit produces
`warning`, not `pass`. Comparisons use the recorded `1e-9` absolute tolerance
to avoid classifying floating-point roundoff at an exact limit as a violation.

## Mapping completeness

The report compares Motion IR joints with the ordered controllable-joint list in
the selected robot profile. It records mapped, missing, and unknown joints plus
the exact coverage ratio.

The default CLI writes incomplete reports because they are useful evidence:

```bash
ohmc quality-report motion.json \
  --robot profiles/unitree_g1_29dof.yaml \
  --output quality.json
```

Release or benchmark jobs can turn missing evidence into a hard gate:

```bash
ohmc quality-report motion.json \
  --robot profiles/unitree_g1_29dof.yaml \
  --output quality.json \
  --require-complete-mapping \
  --require-dynamic-limits
```

These flags affect the command exit code without deleting the generated report.
They do not enable hardware transport.
