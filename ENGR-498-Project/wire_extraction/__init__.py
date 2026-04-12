from __future__ import annotations

from .stitching import WireStitchingParameters, stitch_wire_clusters


def main() -> int:
    from .pipeline import main as _main

    return _main()


def run_wire_extraction(*args, **kwargs):
    from .pipeline import run_wire_extraction as _run_wire_extraction

    return _run_wire_extraction(*args, **kwargs)


__all__ = ["main", "run_wire_extraction", "WireStitchingParameters", "stitch_wire_clusters"]
