import json
from pathlib import Path

from ohmc.audit import validate_evidence_audit, verify_evidence
from ohmc.cli import main


ROOT = Path(__file__).resolve().parents[1]
BUNDLE_SCHEMA = json.loads(
    (ROOT / "schemas" / "simulation-bundle-v0.1.schema.json").read_text()
)
MATRIX_SCHEMA = json.loads(
    (ROOT / "schemas" / "simulation-matrix-v0.1.schema.json").read_text()
)
AUDIT_SCHEMA = json.loads(
    (ROOT / "schemas" / "evidence-audit-v0.1.schema.json").read_text()
)


def simulation_args(output: Path, cache: Path, target: str) -> list[str]:
    return [
        "simulate",
        str(ROOT / "examples" / "simple_motion.bvh"),
        "--target",
        target,
        "--source-license",
        "CC0-1.0",
        "--source-convention",
        "right_handed_x_right_y_up_z_backward",
        "--source-length-unit",
        "m",
        "--output",
        str(output),
        "--cache-dir",
        str(cache),
    ]


def audit(path: Path) -> dict:
    return verify_evidence(
        path, bundle_schema=BUNDLE_SCHEMA, matrix_schema=MATRIX_SCHEMA
    )


def test_bundle_integrity_audit_detects_artifact_tampering(tmp_path: Path) -> None:
    bundle = tmp_path / "bundle"
    assert main(
        simulation_args(bundle, tmp_path / "cache", "unitree-g1-contract-fixture")
    ) == 0

    clean = audit(bundle)

    assert validate_evidence_audit(clean, AUDIT_SCHEMA) == []
    assert clean["status"] == "pass"
    assert clean["kind"] == "simulation_bundle"
    assert clean["checked_bundle_count"] == 1
    assert clean["checked_artifact_count"] == len(
        json.loads((bundle / "manifest.json").read_text())["artifacts"]
    )

    motion = bundle / "motion.json"
    motion.write_text(motion.read_text() + " ")
    tampered = audit(bundle)
    assert tampered["status"] == "fail"
    assert any("artifact motion: SHA-256 mismatch" in issue for issue in tampered["issues"])


def test_failed_execution_matrix_can_still_pass_integrity_audit(tmp_path: Path) -> None:
    matrix = tmp_path / "matrix"
    assert main(simulation_args(matrix, tmp_path / "empty-cache", "all")) == 1

    report_path = tmp_path / "audit.json"
    assert main(
        ["verify-evidence", str(matrix), "--report", str(report_path)]
    ) == 0
    report = json.loads(report_path.read_text())

    assert report["status"] == "pass"
    assert report["kind"] == "simulation_matrix"
    assert report["checked_bundle_count"] == 2
    assert report["issues"] == []


def test_unknown_evidence_directory_fails_with_preserved_report(tmp_path: Path) -> None:
    report = audit(tmp_path)

    assert validate_evidence_audit(report, AUDIT_SCHEMA) == []
    assert report["kind"] == "unknown"
    assert report["status"] == "fail"
    assert report["root_manifest_sha256"] is None
