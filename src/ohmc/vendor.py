"""Pinned vendor SDK acquisition and verification."""

from __future__ import annotations

import hashlib
import os
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import yaml

from .errors import OhmcError


@dataclass(frozen=True)
class Component:
    vendor: str
    name: str
    config: dict[str, Any]

    @property
    def acquisition(self) -> str:
        return str(self.config.get("acquisition", ""))


@dataclass(frozen=True)
class ComponentStatus:
    component: Component
    state: str
    detail: str


def normalize_vendor_name(name: str) -> str:
    return name.strip().lower().replace("-", "_")


def default_cache_dir() -> Path:
    configured = os.environ.get("OHMC_CACHE_DIR")
    if configured:
        return Path(configured).expanduser()
    return Path.home() / ".cache" / "ohmc"


def load_vendor_lock(path: Path) -> dict[str, Any]:
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise OhmcError(f"vendor lock not found: {path}") from exc
    except yaml.YAMLError as exc:
        raise OhmcError(f"invalid vendor lock YAML in {path}: {exc}") from exc

    if not isinstance(data, dict) or data.get("schema") != "ohmc.vendor_lock/v0.1":
        raise OhmcError("unsupported or missing vendor lock schema")
    vendors = data.get("vendors")
    if not isinstance(vendors, dict) or not vendors:
        raise OhmcError("vendor lock must define at least one vendor")
    for vendor_name, vendor_config in vendors.items():
        if not isinstance(vendor_config, dict):
            raise OhmcError(f"vendor {vendor_name!r} must be an object")
        components = vendor_config.get("components")
        if not isinstance(components, dict) or not components:
            raise OhmcError(f"vendor {vendor_name!r} must define components")
        for component_name, component_config in components.items():
            if not isinstance(component_config, dict):
                raise OhmcError(
                    f"component {vendor_name}.{component_name} must be an object"
                )
            acquisition = component_config.get("acquisition")
            if acquisition not in {"git", "official_download", "system"}:
                raise OhmcError(
                    f"component {vendor_name}.{component_name} has unsupported "
                    f"acquisition mode {acquisition!r}"
                )
    return data


def iter_components(
    lock: dict[str, Any], vendor_filter: str | None = None
) -> Iterable[Component]:
    normalized_filter = normalize_vendor_name(vendor_filter) if vendor_filter else None
    vendors: dict[str, Any] = lock["vendors"]
    if normalized_filter and normalized_filter not in vendors:
        available = ", ".join(sorted(vendors))
        raise OhmcError(
            f"unknown vendor {vendor_filter!r}; available vendors: {available}"
        )
    for vendor_name, vendor_config in vendors.items():
        if normalized_filter and vendor_name != normalized_filter:
            continue
        for component_name, component_config in vendor_config["components"].items():
            yield Component(vendor_name, component_name, component_config)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def artifact_cache_path(cache_dir: Path, component: Component) -> Path:
    artifact_name = component.config.get("artifact_name")
    if not artifact_name:
        raise OhmcError(
            f"component {component.vendor}.{component.name} has no artifact_name"
        )
    return cache_dir / "artifacts" / component.vendor / component.name / str(artifact_name)


def git_cache_path(cache_dir: Path, component: Component) -> Path:
    return cache_dir / "sources" / component.vendor / component.name


def _run_git(args: list[str], cwd: Path | None = None) -> str:
    try:
        completed = subprocess.run(
            ["git", *args],
            cwd=cwd,
            check=True,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
    except FileNotFoundError as exc:
        raise OhmcError("git executable not found") from exc
    except subprocess.CalledProcessError as exc:
        detail = exc.stderr.strip() or exc.stdout.strip() or f"exit code {exc.returncode}"
        raise OhmcError(f"git {' '.join(args)} failed: {detail}") from exc
    return completed.stdout.strip()


def component_status(cache_dir: Path, component: Component) -> ComponentStatus:
    if component.acquisition == "official_download":
        path = artifact_cache_path(cache_dir, component)
        if not path.is_file():
            return ComponentStatus(component, "missing", str(path))
        expected = str(component.config.get("sha256", ""))
        actual = sha256_file(path)
        if not expected:
            return ComponentStatus(component, "invalid-lock", "missing SHA-256")
        if actual != expected:
            return ComponentStatus(
                component, "checksum-mismatch", f"expected {expected}, got {actual}"
            )
        return ComponentStatus(component, "verified", actual)

    if component.acquisition == "git":
        path = git_cache_path(cache_dir, component)
        if not (path / ".git").exists():
            return ComponentStatus(component, "missing", str(path))
        try:
            actual = _run_git(["rev-parse", "HEAD"], cwd=path)
        except OhmcError as exc:
            return ComponentStatus(component, "invalid", str(exc))
        expected = str(component.config.get("revision", ""))
        if actual != expected:
            return ComponentStatus(
                component, "revision-mismatch", f"expected {expected}, got {actual}"
            )
        return ComponentStatus(component, "verified", actual)

    return ComponentStatus(component, "system", "version detection not implemented")


def status_all(
    lock: dict[str, Any], cache_dir: Path, vendor_filter: str | None = None
) -> list[ComponentStatus]:
    return [
        component_status(cache_dir, component)
        for component in iter_components(lock, vendor_filter)
    ]


def import_official_artifact(
    lock: dict[str, Any], cache_dir: Path, vendor_name: str, source: Path
) -> Path:
    if not source.is_file():
        raise OhmcError(f"artifact not found: {source}")
    candidates = [
        component
        for component in iter_components(lock, vendor_name)
        if component.acquisition == "official_download"
        and component.config.get("artifact_name") == source.name
    ]
    if not candidates:
        expected_names = sorted(
            str(component.config.get("artifact_name"))
            for component in iter_components(lock, vendor_name)
            if component.acquisition == "official_download"
        )
        raise OhmcError(
            f"artifact name {source.name!r} is not locked for {vendor_name}; "
            f"expected one of: {', '.join(expected_names)}"
        )
    component = candidates[0]
    expected = str(component.config.get("sha256", ""))
    actual = sha256_file(source)
    if actual != expected:
        raise OhmcError(
            f"SHA-256 mismatch for {source.name}: expected {expected}, got {actual}"
        )

    destination = artifact_cache_path(cache_dir, component)
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists() and sha256_file(destination) == expected:
        return destination
    temporary = destination.with_suffix(destination.suffix + ".partial")
    shutil.copy2(source, temporary)
    if sha256_file(temporary) != expected:
        temporary.unlink(missing_ok=True)
        raise OhmcError(f"copied artifact failed verification: {temporary}")
    temporary.replace(destination)
    return destination


def sync_git_vendor(
    lock: dict[str, Any], cache_dir: Path, vendor_name: str
) -> list[Path]:
    synced: list[Path] = []
    components = list(iter_components(lock, vendor_name))
    git_components = [item for item in components if item.acquisition == "git"]
    if not git_components:
        raise OhmcError(f"vendor {vendor_name!r} has no Git components")

    for component in git_components:
        source = str(component.config.get("source", ""))
        revision = str(component.config.get("revision", ""))
        if not source or not revision:
            raise OhmcError(
                f"component {component.vendor}.{component.name} lacks source or revision"
            )
        destination = git_cache_path(cache_dir, component)
        if destination.exists() and not (destination / ".git").exists():
            raise OhmcError(f"refusing to overwrite non-Git path: {destination}")
        if not destination.exists():
            destination.parent.mkdir(parents=True, exist_ok=True)
            _run_git(["clone", "--filter=blob:none", source, str(destination)])
        _run_git(["fetch", "--depth", "1", "origin", revision], cwd=destination)
        _run_git(["checkout", "--detach", revision], cwd=destination)
        actual = _run_git(["rev-parse", "HEAD"], cwd=destination)
        if actual != revision:
            raise OhmcError(
                f"component {component.vendor}.{component.name} checked out {actual}, "
                f"expected {revision}"
            )
        synced.append(destination)
    return synced

