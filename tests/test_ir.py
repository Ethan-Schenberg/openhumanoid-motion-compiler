import json
from pathlib import Path

from ohmc.ir import validate_motion_ir


ROOT = Path(__file__).resolve().parents[1]


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def test_minimal_motion_is_valid() -> None:
    schema = load(ROOT / "schemas" / "motion-ir-v0.1.schema.json")
    motion = load(ROOT / "examples" / "minimal_motion.json")

    assert validate_motion_ir(motion, schema) == []


def test_semantic_validation_catches_joint_count_and_time_order() -> None:
    schema = load(ROOT / "schemas" / "motion-ir-v0.1.schema.json")
    motion = load(ROOT / "examples" / "minimal_motion.json")
    motion["trajectory"]["samples"][1]["time"] = 0.0
    motion["trajectory"]["samples"][1]["position_targets"] = [0.1]

    issues = validate_motion_ir(motion, schema)

    assert any("strictly greater" in issue for issue in issues)
    assert any("expected 2 values" in issue for issue in issues)

