import hashlib
from pathlib import Path

import yaml

from ohmc.vendor import (
    import_official_artifact,
    load_vendor_lock,
    doctor_report,
    status_all,
)


def write_lock(path: Path, artifact_name: str, checksum: str, *, license_name: str = "MIT", redistribute: bool = False) -> None:
    data = {
        "schema": "ohmc.vendor_lock/v0.1",
        "vendors": {
            "example_robot": {
                "components": {
                    "sdk": {
                        "version": "1.0.0",
                        "artifact_name": artifact_name,
                        "sha256": checksum,
                        "acquisition": "official_download",
                        "license": license_name,
                        "redistribute": redistribute,
                    }
                }
            }
        },
    }
    path.write_text(yaml.safe_dump(data), encoding="utf-8")


def test_import_and_verify_official_artifact(tmp_path: Path) -> None:
    artifact = tmp_path / "example-sdk.zip"
    artifact.write_bytes(b"official SDK fixture")
    checksum = hashlib.sha256(artifact.read_bytes()).hexdigest()
    lock_path = tmp_path / "vendor-lock.yaml"
    write_lock(lock_path, artifact.name, checksum)
    lock = load_vendor_lock(lock_path)
    cache_dir = tmp_path / "cache"

    destination = import_official_artifact(
        lock, cache_dir, "example-robot", artifact
    )
    statuses = status_all(lock, cache_dir, "example_robot")

    assert destination.read_bytes() == artifact.read_bytes()
    assert len(statuses) == 1
    assert statuses[0].state == "verified"


def test_import_rejects_checksum_mismatch(tmp_path: Path) -> None:
    artifact = tmp_path / "example-sdk.zip"
    artifact.write_bytes(b"unexpected bytes")
    lock_path = tmp_path / "vendor-lock.yaml"
    write_lock(lock_path, artifact.name, "0" * 64)
    lock = load_vendor_lock(lock_path)

    try:
        import_official_artifact(lock, tmp_path / "cache", "example_robot", artifact)
    except Exception as exc:
        assert "SHA-256 mismatch" in str(exc)
    else:
        raise AssertionError("checksum mismatch was accepted")


def test_doctor_report_marks_resolved_and_unresolved_states(tmp_path: Path) -> None:
    artifact = tmp_path / "example-sdk.zip"
    artifact.write_bytes(b"official SDK fixture")
    checksum = hashlib.sha256(artifact.read_bytes()).hexdigest()
    lock_path = tmp_path / "vendor-lock.yaml"
    write_lock(
        lock_path,
        artifact.name,
        checksum,
        license_name="unresolved",
        redistribute=False,
    )
    lock = load_vendor_lock(lock_path)
    cache_dir = tmp_path / "cache"
    import_official_artifact(lock, cache_dir, "example_robot", artifact)

    report = doctor_report(lock, cache_dir, "example_robot")
    assert report["healthy"] is True
    assert report["critical"] == 0
    assert report["warning"] == 2  # unresolved license + redistribution disabled
    vendor_payload = report["vendors"]["example_robot"]
    assert vendor_payload["critical"] == 0
    assert vendor_payload["warning"] == 2
    assert vendor_payload["components"][0]["component"] == "sdk"
