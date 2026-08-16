import json
from pathlib import Path

import pytest

from ohmc.bvh import bvh_to_motion_ir, load_bvh
from ohmc.errors import OhmcError
from ohmc.profiles import (
    load_yaml_object,
    map_motion_ir,
    validate_robot_profile,
    validate_semantic_map,
)


ROOT = Path(__file__).resolve().parents[1]
PROFILE_SCHEMA = json.loads(
    (ROOT / "schemas" / "robot-profile-v0.1.schema.json").read_text()
)
MAPPING_SCHEMA = json.loads(
    (ROOT / "schemas" / "semantic-map-v0.1.schema.json").read_text()
)


def source_motion() -> dict:
    source = ROOT / "examples" / "simple_motion.bvh"
    return bvh_to_motion_ir(
        load_bvh(source), source.read_bytes(), source.name, "CC0-1.0"
    )


@pytest.mark.parametrize(
    ("filename", "joint_count", "excluded"),
    [
        ("unitree_g1_29dof.yaml", 29, []),
        ("agibot_x2_ultra_aimdk_v1.yaml", 30, ["head_pitch_joint"]),
    ],
)
def test_vendor_profiles_are_valid(
    filename: str, joint_count: int, excluded: list[str]
) -> None:
    profile = load_yaml_object(ROOT / "profiles" / filename)

    assert validate_robot_profile(profile, PROFILE_SCHEMA) == []
    assert len(profile["control"]["joint_order"]) == joint_count
    assert [item["name"] for item in profile["control"]["excluded_joints"]] == excluded
    assert profile["control"]["hardware_transport"] == "disabled"


@pytest.mark.parametrize(
    "filename",
    ["unitree_g1_29dof.yaml", "agibot_x2_ultra_aimdk_v1.yaml"],
)
def test_semantic_map_produces_profile_order_and_safe_knee(filename: str) -> None:
    profile = load_yaml_object(ROOT / "profiles" / filename)
    mapping = load_yaml_object(
        ROOT / "profiles" / "mappings" / "simple_bvh_semantics_v1.yaml"
    )

    assert validate_semantic_map(mapping, MAPPING_SCHEMA) == []
    mapped = map_motion_ir(source_motion(), profile, mapping)

    assert mapped["trajectory"]["joints"] == [
        "left_knee_joint",
        "waist_yaw_joint",
    ]
    assert mapped["trajectory"]["samples"][2]["position_targets"] == [
        pytest.approx(0.17453292519943295),
        pytest.approx(0.06981317007977318),
    ]
    assert mapped["robot"]["profile"] == profile["id"]
    assert mapped["passes"][-1]["name"] == "semantic_joint_map"
    assert "hardware transport remains disabled" in mapped["validation"]["issues"][-2]


def test_mapping_rejects_profile_limit_violation() -> None:
    profile = load_yaml_object(ROOT / "profiles" / "agibot_x2_ultra_aimdk_v1.yaml")
    mapping = load_yaml_object(
        ROOT / "profiles" / "mappings" / "simple_bvh_semantics_v1.yaml"
    )
    motion = source_motion()
    motion["trajectory"]["samples"][1]["position_targets"][3] = -10.0

    with pytest.raises(OhmcError, match="violates profile range"):
        map_motion_ir(motion, profile, mapping)
