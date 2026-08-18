# Canonical Motion v0.1

`ohmc.canonical_motion/v0.1` is the source-side skeleton contract between BVH
parsing and future morphology/IK passes. It is not a robot command trajectory.

## Coordinate contract

All output uses a right-handed frame:

- `+X` forward
- `+Y` left
- `+Z` up
- positions and offsets in metres
- rotations as normalized quaternions in `x, y, z, w` order
- timestamps in seconds from the first source frame

BVH does not carry a dependable machine-readable declaration of axis direction
or length unit. The CLI therefore requires both `--source-convention` and
`--source-length-unit`; it never infers them from a visually plausible pose.

The initial source conventions are:

- `right_handed_x_forward_y_left_z_up`: already canonical.
- `right_handed_x_right_y_up_z_backward`: converted as
  `(x, y, z) -> (-z, -x, y)`.

Supported source length units are `m`, `cm`, and `mm`.

## Transform evaluation

For each joint and frame, OHMC:

1. reads position and rotation values from the joint's declared channel order;
2. multiplies axis rotations in that declared order;
3. changes basis into the canonical frame;
4. combines the scaled joint offset with local position channels;
5. runs parent-to-child forward kinematics;
6. emits local translations and rotations plus world positions and rotations.

The local translations are part of the contract, rather than an implementation
detail. This lets later passes change skeleton morphology or sampling rate and
then recompute world poses instead of scaling already-evaluated coordinates.

## Morphology and timeline normalization

`ohmc normalize-canonical` applies a uniform positive morphology scale to rest
offsets, root translation, and per-joint local translations. It resamples the
source interval at an explicit target rate using linear translation
interpolation and shortest-arc quaternion SLERP. World poses are discarded and
recomputed from the interpolated local poses with deterministic forward
kinematics.

The source's first and final timestamps are always preserved. If the duration
is not an integer number of target intervals, a final shorter interval is
emitted and recorded as a pass warning rather than truncating the motion or
silently changing its duration. The pass records its configuration, input, and
output hashes plus input/output sample counts, duration, target rate, and
morphology scale. Adjacent pass hashes form an auditable content chain.

The artifact records its source SHA-256, convention, unit conversion, pass
configuration hash, joint/frame counts, and validation result. Validation checks
schema shape, unique joint names, parent ordering, monotonic time, finite values,
per-frame joint counts, and unit quaternion norms.

## Current boundary

Canonical source kinematics, morphology scaling, and timeline resampling are
implemented, but the existing semantic joint-vector mapper does not yet solve
against the canonical world poses. Landmark semantics, constrained whole-body
IK, contact preservation, and dynamic control remain later compiler passes.
Consequently,
the one-command bundle can report canonical source motion as `pass` while its
robot Motion IR still correctly reports a mapping warning.
