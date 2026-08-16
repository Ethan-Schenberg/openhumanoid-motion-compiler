import json
from pathlib import Path

import pytest

from ohmc.bvh import bvh_to_motion_ir, load_bvh
from ohmc.errors import OhmcError
from ohmc.replay import replay_mujoco


mujoco = pytest.importorskip("mujoco")
ROOT = Path(__file__).resolve().parents[1]


def make_motion() -> dict:
    source = ROOT / "examples" / "simple_motion.bvh"
    return bvh_to_motion_ir(
        load_bvh(source), source.read_bytes(), source.name, "CC0-1.0"
    )


def test_headless_mujoco_replay_maps_all_joints() -> None:
    report = replay_mujoco(make_motion(), ROOT / "examples" / "simple_chain.xml")

    assert report == {
        "backend": "mujoco",
        "mode": "headless_kinematic_mj_forward",
        "model": "simple_chain.xml",
        "frames_replayed": 3,
        "joints_mapped": 6,
        "duration_seconds": pytest.approx(0.04),
        "maximum_absolute_position": pytest.approx(0.17453292519943295),
        "hardware_commands_sent": False,
        "status": "pass",
    }


def test_headless_mujoco_replay_rejects_unknown_joint(tmp_path: Path) -> None:
    document = make_motion()
    document = json.loads(json.dumps(document))
    document["trajectory"]["joints"][0] = "missing_joint"

    with pytest.raises(OhmcError, match="not found"):
        replay_mujoco(document, ROOT / "examples" / "simple_chain.xml")
