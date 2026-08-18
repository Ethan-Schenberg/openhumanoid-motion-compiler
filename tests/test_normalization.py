import json
import math
from pathlib import Path

import pytest

from ohmc.bvh import load_bvh
from ohmc.canonical import BVH_Y_UP_CONVENTION, bvh_to_canonical_motion
from ohmc.cli import main
from ohmc.errors import OhmcError
from ohmc.normalization import normalize_canonical_motion, quaternion_slerp


ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT / "examples" / "simple_motion.bvh"


def canonical_example() -> dict:
    return bvh_to_canonical_motion(
        load_bvh(EXAMPLE),
        source_bytes=EXAMPLE.read_bytes(),
        source_name=EXAMPLE.name,
        source_license="CC0-1.0",
        source_convention=BVH_Y_UP_CONVENTION,
        source_length_unit="m",
    )


def test_normalization_scales_local_geometry_and_recomputes_fk() -> None:
    source = canonical_example()
    original = json.dumps(source, sort_keys=True)

    normalized = normalize_canonical_motion(
        source, morphology_scale=2.0, rate_hz=50.0
    )

    assert json.dumps(source, sort_keys=True) == original
    assert normalized["skeleton"]["joints"][1]["rest_offset_m"] == [0.0, 0.0, -0.8]
    for source_position, output_position in zip(
        source["samples"][2]["world_positions_m"],
        normalized["samples"][2]["world_positions_m"],
    ):
        assert output_position == pytest.approx(
            [2.0 * value for value in source_position], abs=1e-12
        )
    for source_rotation, output_rotation in zip(
        source["samples"][2]["local_rotations_xyzw"],
        normalized["samples"][2]["local_rotations_xyzw"],
    ):
        assert output_rotation == pytest.approx(source_rotation, abs=1e-12)
    assert normalized["passes"][-1]["metrics"]["morphology_scale"] == 2.0
    assert (
        normalized["passes"][0]["output_sha256"]
        == normalized["passes"][1]["input_sha256"]
    )


def test_resampling_uses_slerp_and_preserves_exact_duration() -> None:
    source = canonical_example()

    normalized = normalize_canonical_motion(
        source, morphology_scale=1.0, rate_hz=100.0
    )

    assert [sample["time"] for sample in normalized["samples"]] == pytest.approx(
        [0.0, 0.01, 0.02, 0.03, 0.04]
    )
    expected = quaternion_slerp(
        source["samples"][0]["local_rotations_xyzw"][0],
        source["samples"][1]["local_rotations_xyzw"][0],
        0.5,
    )
    assert normalized["samples"][1]["local_rotations_xyzw"][0] == pytest.approx(
        expected, abs=1e-12
    )
    assert normalized["samples"][1]["local_rotations_xyzw"][0] == pytest.approx(
        [-math.sin(math.radians(0.5)), 0.0, 0.0, math.cos(math.radians(0.5))],
        abs=1e-12,
    )


def test_resampling_appends_non_grid_final_timestamp_with_warning() -> None:
    source = canonical_example()
    source["samples"][-1]["time"] = 0.045

    normalized = normalize_canonical_motion(
        source, morphology_scale=1.0, rate_hz=50.0
    )

    assert [sample["time"] for sample in normalized["samples"]] == pytest.approx(
        [0.0, 0.02, 0.04, 0.045]
    )
    assert normalized["passes"][-1]["warnings"]


def test_slerp_treats_opposite_quaternion_signs_as_same_rotation() -> None:
    quaternion = [0.0, math.sin(math.radians(30.0)), 0.0, math.cos(math.radians(30.0))]
    opposite = [-value for value in quaternion]

    midpoint = quaternion_slerp(quaternion, opposite, 0.5)

    assert midpoint == pytest.approx(quaternion, abs=1e-12)


def test_single_sample_motion_is_scaled_without_inventing_timestamps() -> None:
    source = canonical_example()
    source["samples"] = source["samples"][:1]

    normalized = normalize_canonical_motion(
        source, morphology_scale=0.5, rate_hz=120.0
    )

    assert [sample["time"] for sample in normalized["samples"]] == [0.0]
    assert normalized["samples"][0]["world_positions_m"][1] == pytest.approx(
        [0.0, 0.0, 0.25]
    )
    assert normalized["passes"][-1]["warnings"] == []


@pytest.mark.parametrize(
    ("scale", "rate"),
    [(0.0, 50.0), (-1.0, 50.0), (1.0, 0.0), (1.0, math.inf)],
)
def test_normalization_rejects_invalid_configuration(scale: float, rate: float) -> None:
    with pytest.raises(OhmcError):
        normalize_canonical_motion(
            canonical_example(), morphology_scale=scale, rate_hz=rate
        )


def test_normalize_cli_writes_and_refuses_overwrite(tmp_path: Path) -> None:
    canonical = tmp_path / "canonical.json"
    normalized = tmp_path / "normalized.json"
    assert (
        main(
            [
                "canonicalize-bvh",
                str(EXAMPLE),
                "--source-license",
                "CC0-1.0",
                "--source-convention",
                BVH_Y_UP_CONVENTION,
                "--source-length-unit",
                "m",
                "--output",
                str(canonical),
            ]
        )
        == 0
    )
    args = [
        "normalize-canonical",
        str(canonical),
        "--rate-hz",
        "100",
        "--morphology-scale",
        "0.9",
        "--output",
        str(normalized),
    ]
    assert main(args) == 0
    output = json.loads(normalized.read_text())
    assert len(output["samples"]) == 5
    assert output["passes"][-1]["metrics"]["morphology_scale"] == 0.9
    assert main(args) == 2
    assert main([*args, "--force"]) == 0
