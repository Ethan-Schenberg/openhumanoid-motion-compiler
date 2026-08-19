"""OHMC X2 task registration hook for Isaac Lab."""

from __future__ import annotations

import sys


def register() -> list[str]:
    """Register X2 environments and return untouched Hydra/preset arguments."""

    from . import tasks  # noqa: F401

    return sys.argv[1:]


__all__ = ["register"]
