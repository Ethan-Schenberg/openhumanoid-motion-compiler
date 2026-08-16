"""OHMC command-line interface."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .errors import OhmcError
from .ir import load_json, validate_motion_ir
from .vendor import (
    default_cache_dir,
    import_official_artifact,
    load_vendor_lock,
    status_all,
    sync_git_vendor,
)


def default_project_root() -> Path:
    return Path(__file__).resolve().parents[2]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="ohmc")
    subcommands = parser.add_subparsers(dest="command", required=True)

    validate_ir = subcommands.add_parser("validate-ir", help="validate Motion IR")
    validate_ir.add_argument("document", type=Path)
    validate_ir.add_argument(
        "--schema",
        type=Path,
        default=default_project_root() / "schemas" / "motion-ir-v0.1.schema.json",
    )

    vendor = subcommands.add_parser("vendor", help="manage vendor SDK dependencies")
    vendor.add_argument(
        "--lock",
        type=Path,
        default=default_project_root() / "vendor" / "vendor-lock.yaml",
    )
    vendor.add_argument("--cache-dir", type=Path, default=default_cache_dir())
    vendor_commands = vendor.add_subparsers(dest="vendor_command", required=True)

    status = vendor_commands.add_parser("status", help="show dependency status")
    status.add_argument("vendor", nargs="?")

    verify = vendor_commands.add_parser("verify", help="verify installed dependencies")
    verify.add_argument("vendor", nargs="?")

    import_command = vendor_commands.add_parser(
        "import", help="import an official downloaded artifact"
    )
    import_command.add_argument("vendor")
    import_command.add_argument("artifact", type=Path)

    sync = vendor_commands.add_parser("sync", help="sync pinned Git dependencies")
    sync.add_argument("vendor")
    return parser


def _print_statuses(statuses: list, strict: bool) -> int:
    print(f"{'COMPONENT':<28} {'STATE':<20} DETAIL")
    failed = False
    for status in statuses:
        component_name = f"{status.component.vendor}.{status.component.name}"
        print(f"{component_name:<28} {status.state:<20} {status.detail}")
        if status.state not in {"verified", "system"}:
            failed = True
    return 1 if strict and failed else 0


def run(args: argparse.Namespace) -> int:
    if args.command == "validate-ir":
        document = load_json(args.document)
        schema = load_json(args.schema)
        issues = validate_motion_ir(document, schema)
        if issues:
            for issue in issues:
                print(f"ERROR {issue}")
            return 1
        print(f"valid Motion IR: {args.document}")
        return 0

    lock = load_vendor_lock(args.lock)
    cache_dir = args.cache_dir.expanduser().resolve()
    if args.vendor_command == "status":
        return _print_statuses(status_all(lock, cache_dir, args.vendor), strict=False)
    if args.vendor_command == "verify":
        return _print_statuses(status_all(lock, cache_dir, args.vendor), strict=True)
    if args.vendor_command == "import":
        destination = import_official_artifact(
            lock, cache_dir, args.vendor, args.artifact.expanduser().resolve()
        )
        print(f"imported and verified: {destination}")
        return 0
    if args.vendor_command == "sync":
        for destination in sync_git_vendor(lock, cache_dir, args.vendor):
            print(f"synced and verified: {destination}")
        return 0
    raise OhmcError("unhandled command")


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return run(args)
    except OhmcError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

