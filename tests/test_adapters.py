import json
from pathlib import Path

import pytest
from jsonschema import Draft202012Validator

from ohmc.adapters import (
    AGIBOT_X2_ADAPTER,
    UNITREE_G1_ADAPTER,
    encode_vendor_fixture,
)
from ohmc.bvh import bvh_to_motion_ir, load_bvh
from ohmc.cli import main
from ohmc.errors import OhmcError
from ohmc.profiles import load_yaml_object, map_motion_ir


ROOT = Path(__file__).resolve().parents[1]
FIXTURE_SCHEMA = json.loads(
    (ROOT / "schemas" / "vendor-interface-fixture-v0.1.schema.json").read_text()
)


def mapped_motion(profile_name: str) -> tuple[dict, dict]:
    source = ROOT / "examples" / "simple_motion.bvh"
    motion = bvh_to_motion_ir(
        load_bvh(source), source.read_bytes(), source.name, "CC0-1.0"
    )
    profile = load_yaml_object(ROOT / "profiles" / profile_name)
    mapping = load_yaml_object(
        ROOT / "profiles" / "mappings" / "simple_bvh_semantics_v1.yaml"
    )
    return map_motion_ir(motion, profile, mapping), profile


def assert_safe_fixture(fixture: dict) -> None:
    assert list(Draft202012Validator(FIXTURE_SCHEMA).iter_errors(fixture)) == []
    assert fixture["transport"] == "disabled"
    assert fixture["executable"] is False
    assert fixture["complete"] is False


def test_unitree_lowcmd_fixture_uses_official_29_slot_order() -> None:
    motion, profile = mapped_motion("unitree_g1_29dof.yaml")
    fixture = encode_vendor_fixture(motion, profile, UNITREE_G1_ADAPTER)
    assert_safe_fixture(fixture)

    commands = fixture["frames"][2]["message"]["motor_cmd"]
    assert len(commands) == 29
    assert [item["name"] for item in commands] == profile["control"]["joint_order"]
    assert commands[3]["q"] == pytest.approx(0.17453292519943295)
    assert commands[12]["q"] == pytest.approx(0.06981317007977318)
    assert [item["index"] for item in commands if item["present"]] == [3, 12]
    for command in commands:
        assert command["mode"] is None
        assert command["dq"] is None
        assert command["kp"] is None
        assert command["kd"] is None
        assert command["tau"] is None
        if not command["present"]:
            assert command["q"] is None


def test_agibot_fixture_uses_aimdk_group_contract_and_exclusion() -> None:
    motion, profile = mapped_motion("agibot_x2_ultra_aimdk_v1.yaml")
    fixture = encode_vendor_fixture(motion, profile, AGIBOT_X2_ADAPTER)
    assert_safe_fixture(fixture)

    messages = fixture["frames"][2]["messages"]
    assert [item["group"] for item in messages] == ["leg", "waist", "arm", "head"]
    assert [len(item["joints"]) for item in messages] == [12, 3, 14, 2]
    leg, waist, _, head = messages
    assert leg["joints"][3]["position"] == pytest.approx(0.17453292519943295)
    assert waist["joints"][0]["position"] == pytest.approx(0.06981317007977318)
    assert head["joints"][1] == {
        "index": 1,
        "name": "head_pitch_joint",
        "available": False,
        "present": False,
        "position": None,
        "velocity": None,
        "effort": None,
        "stiffness": None,
        "damping": None,
    }
    for message in messages:
        assert message["header"] is None
        for joint in message["joints"]:
            assert joint["velocity"] is None
            assert joint["effort"] is None
            assert joint["stiffness"] is None
            assert joint["damping"] is None


def test_adapter_rejects_wrong_profile() -> None:
    motion, profile = mapped_motion("unitree_g1_29dof.yaml")
    with pytest.raises(OhmcError, match="requires robot profile"):
        encode_vendor_fixture(motion, profile, AGIBOT_X2_ADAPTER)


def test_encode_fixture_cli_writes_and_refuses_overwrite(tmp_path: Path) -> None:
    motion, _ = mapped_motion("unitree_g1_29dof.yaml")
    source = tmp_path / "motion.json"
    output = tmp_path / "fixture.json"
    source.write_text(json.dumps(motion), encoding="utf-8")
    args = [
        "encode-fixture",
        str(source),
        "--robot",
        str(ROOT / "profiles" / "unitree_g1_29dof.yaml"),
        "--adapter",
        UNITREE_G1_ADAPTER,
        "--output",
        str(output),
    ]
    assert main(args) == 0
    assert json.loads(output.read_text())["executable"] is False
    assert main(args) == 2
    assert main([*args, "--force"]) == 0
