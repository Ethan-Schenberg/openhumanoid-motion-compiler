# Example provenance

`simple_motion.bvh` is an original synthetic fixture created for OHMC. It is
released under CC0-1.0 and contains no captured human performance or vendor
robot data.

`full_body_motion.bvh` is an original synthetic CC0-1.0 fixture created for
OHMC. It defines 16 named pelvis, spine, head, bilateral arm, and bilateral leg
landmarks with 51 channels. Its small analytic motion is designed for coverage,
symmetry, IK, and contact-regression tests; it is not a captured performance.

`minimal_motion.json` is also synthetic and released under CC0-1.0, as recorded
inside the artifact.

`semantic_contract_fixture.xml` is an original synthetic MuJoCo model released
under the repository's Apache-2.0 license. It contains only
`waist_yaw_joint` and `left_knee_joint`; it tests one-command orchestration and
vendor interface ordering, not Unitree G1 or AgiBot X2 geometry or dynamics.

`ik_contract_fixture.xml` is an original Apache-2.0 single-hinge MuJoCo model.
Together with `ik_task_map_fixture.yaml`, it provides an analytically checkable
bounded-IK test whose expected roll sequence is 0, -2, and -4 degrees.
