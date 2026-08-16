import json
import math
from pathlib import Path

import pytest

from ohmc.bvh import bvh_to_motion_ir, load_bvh, parse_bvh_text
from ohmc.cli import main
from ohmc.errors import OhmcError
from ohmc.ir import validate_motion_ir


ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT / "examples" / "simple_motion.bvh"


def test_parse_bvh_hierarchy_timing_and_samples() -> None:
    motion = load_bvh(EXAMPLE)

    assert [joint.name for joint in motion.joints] == ["Hips", "LeftKnee"]
    assert motion.joints[1].parent == "Hips"
    assert len(motion.channel_bindings) == 9
    assert len(motion.frames) == 3
    assert motion.frame_time == pytest.approx(0.02)
    assert motion.duration == pytest.approx(0.04)


def test_bvh_import_emits_radians_and_valid_motion_ir() -> None:
    source_bytes = EXAMPLE.read_bytes()
    motion = load_bvh(EXAMPLE)
    document = bvh_to_motion_ir(
        motion, source_bytes, EXAMPLE.name, source_license="CC0-1.0"
    )
    schema = json.loads(
        (ROOT / "schemas" / "motion-ir-v0.1.schema.json").read_text()
    )

    assert validate_motion_ir(document, schema) == []
    assert document["trajectory"]["rate_hz"] == pytest.approx(50.0)
    assert document["trajectory"]["joints"] == [
        "Hips.z_rotation",
        "Hips.x_rotation",
        "Hips.y_rotation",
        "LeftKnee.z_rotation",
        "LeftKnee.x_rotation",
        "LeftKnee.y_rotation",
    ]
    assert document["trajectory"]["samples"][2]["position_targets"][0] == (
        pytest.approx(math.radians(4.0))
    )
    assert document["trajectory"]["samples"][2]["position_targets"][3] == (
        pytest.approx(math.radians(-10.0))
    )
    assert document["validation"]["status"] == "warning"
    assert "canonical axis remapping" in document["validation"]["issues"][0]
    assert "translation channels" in document["validation"]["issues"][1]
    assert "solver_status" not in document["trajectory"]["samples"][0]


def test_parser_rejects_wrong_frame_value_count() -> None:
    text = EXAMPLE.read_text().rsplit(" 0.0", 1)[0]

    with pytest.raises(OhmcError, match="frame data count mismatch"):
        parse_bvh_text(text)


def test_import_bvh_cli_writes_and_refuses_overwrite(tmp_path: Path) -> None:
    output = tmp_path / "motion.json"
    args = [
        "import-bvh",
        str(EXAMPLE),
        "--source-license",
        "CC0-1.0",
        "--output",
        str(output),
    ]

    assert main(args) == 0
    assert output.exists()
    assert main(args) == 2
    assert main([*args, "--force"]) == 0
