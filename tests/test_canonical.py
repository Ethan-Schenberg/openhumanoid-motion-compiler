import json
import math
from pathlib import Path

import pytest

from ohmc.bvh import load_bvh, parse_bvh_text
from ohmc.canonical import (
    BVH_Y_UP_CONVENTION,
    CANONICAL_CONVENTION,
    bvh_to_canonical_motion,
    validate_canonical_motion,
)
from ohmc.cli import main
from ohmc.errors import OhmcError


ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT / "examples" / "simple_motion.bvh"
SCHEMA = json.loads(
    (ROOT / "schemas" / "canonical-motion-v0.1.schema.json").read_text()
)


def canonical_example() -> dict:
    source_bytes = EXAMPLE.read_bytes()
    return bvh_to_canonical_motion(
        load_bvh(EXAMPLE),
        source_bytes=source_bytes,
        source_name=EXAMPLE.name,
        source_license="CC0-1.0",
        source_convention=BVH_Y_UP_CONVENTION,
        source_length_unit="m",
    )


def test_canonical_forward_kinematics_matches_golden_coordinates() -> None:
    document = canonical_example()

    assert validate_canonical_motion(document, SCHEMA) == []
    assert document["frames"] == {
        "convention": "right_handed_x_forward_y_left_z_up",
        "source_convention": "right_handed_x_right_y_up_z_backward",
        "length_unit": "meter",
    }
    assert document["skeleton"]["joints"] == [
        {"name": "Hips", "parent_index": None, "rest_offset_m": [0.0, 0.0, 0.9]},
        {
            "name": "LeftKnee",
            "parent_index": 0,
            "rest_offset_m": [0.0, 0.0, -0.4],
        },
    ]
    first = document["samples"][0]
    assert first["world_positions_m"] == [[0.0, 0.0, 0.9], [0.0, 0.0, 0.5]]
    assert first["local_rotations_xyzw"] == [
        [0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, 0.0, 1.0],
    ]

    third = document["samples"][2]
    assert third["local_rotations_xyzw"][0] == pytest.approx(
        [-math.sin(math.radians(2.0)), 0.0, 0.0, math.cos(math.radians(2.0))]
    )
    assert third["local_rotations_xyzw"][1] == pytest.approx(
        [math.sin(math.radians(5.0)), 0.0, 0.0, math.cos(math.radians(5.0))]
    )
    assert third["world_positions_m"][1] == pytest.approx(
        [0.0, -0.4 * math.sin(math.radians(4.0)), 0.9 - 0.4 * math.cos(math.radians(4.0))]
    )


def test_source_axis_and_centimeter_conversion_are_explicit() -> None:
    motion = parse_bvh_text(
        """HIERARCHY
ROOT Root
{
  OFFSET 0 0 0
  CHANNELS 6 Xposition Yposition Zposition Xrotation Yrotation Zrotation
}
MOTION
Frames: 1
Frame Time: 0.01
1 2 3 0 0 0
"""
    )
    document = bvh_to_canonical_motion(
        motion,
        source_bytes=b"axis fixture",
        source_name="axis.bvh",
        source_license="CC0-1.0",
        source_convention=BVH_Y_UP_CONVENTION,
        source_length_unit="cm",
    )

    assert document["samples"][0]["root_translation_m"] == pytest.approx(
        [-0.03, -0.01, 0.02]
    )
    assert document["samples"][0]["world_positions_m"][0] == pytest.approx(
        [-0.03, -0.01, 0.02]
    )
    assert document["passes"][0]["metrics"]["source_length_to_meter_scale"] == 0.01


def test_rotation_channels_are_postmultiplied_in_declared_order() -> None:
    motion = parse_bvh_text(
        """HIERARCHY
ROOT Root
{
  OFFSET 0 0 0
  CHANNELS 3 Zrotation Xrotation Yrotation
  JOINT Tip
  {
    OFFSET 0 1 0
    CHANNELS 0
  }
}
MOTION
Frames: 1
Frame Time: 0.01
90 90 0
"""
    )
    document = bvh_to_canonical_motion(
        motion,
        source_bytes=b"rotation-order fixture",
        source_name="rotation-order.bvh",
        source_license="CC0-1.0",
        source_convention=CANONICAL_CONVENTION,
        source_length_unit="m",
    )

    # Rz(90) * Rx(90) maps the child +Y offset to +Z. Reversing the declared
    # channel order would map it to -X, so this catches a visually subtle bug.
    assert document["samples"][0]["world_positions_m"][1] == pytest.approx(
        [0.0, 0.0, 1.0], abs=1e-12
    )


def test_canonicalize_cli_writes_and_refuses_overwrite(tmp_path: Path) -> None:
    output = tmp_path / "canonical.json"
    args = [
        "canonicalize-bvh",
        str(EXAMPLE),
        "--source-license",
        "CC0-1.0",
        "--source-convention",
        BVH_Y_UP_CONVENTION,
        "--source-length-unit",
        "m",
        "--output",
        str(output),
    ]

    assert main(args) == 0
    assert json.loads(output.read_text())["validation"]["status"] == "pass"
    assert main(args) == 2
    assert main([*args, "--force"]) == 0


def test_parser_rejects_duplicate_joint_channels() -> None:
    text = """HIERARCHY
ROOT Root
{
  OFFSET 0 0 0
  CHANNELS 2 Xrotation Xrotation
}
MOTION
Frames: 1
Frame Time: 0.01
0 0
"""
    with pytest.raises(OhmcError, match="duplicate BVH channel"):
        parse_bvh_text(text)
