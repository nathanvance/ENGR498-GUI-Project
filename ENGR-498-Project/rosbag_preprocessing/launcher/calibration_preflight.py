from __future__ import annotations

import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path


RUN_PIPELINE = Path(__file__).with_name("run_pipeline.py")
SOFTWARE_RENDERER_PATTERNS = (
    "llvmpipe",
    "softpipe",
    "software rasterizer",
    "lavapipe",
    "basic render",
)


@dataclass
class PreflightResult:
    ok: bool
    summary: str
    details: list[str]
    renderer: str | None = None
    direct_rendering: str | None = None


def _run(args: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        args,
        check=False,
        text=True,
        capture_output=True,
    )


def _extract_value(output: str, prefix: str) -> str | None:
    pattern = re.compile(rf"^{re.escape(prefix)}\s*:\s*(.+)$", re.MULTILINE)
    match = pattern.search(output)
    return match.group(1).strip() if match else None


def _looks_like_software_renderer(renderer: str | None) -> bool:
    if not renderer:
        return True
    lowered = renderer.lower()
    return any(token in lowered for token in SOFTWARE_RENDERER_PATTERNS)


def run_preflight() -> PreflightResult:
    details: list[str] = []

    try:
        wsl_probe = _run(["wsl", "bash", "-lc", "printf 'ok\\n'"])
    except FileNotFoundError:
        return PreflightResult(
            ok=False,
            summary="WSL is not available on this system.",
            details=[
                "Calibration on Windows requires WSL2.",
                "Install WSL2, then retry the calibration requirements check.",
            ],
        )

    if wsl_probe.returncode != 0 or "ok" not in wsl_probe.stdout:
        return PreflightResult(
            ok=False,
            summary="WSL is installed but not responding correctly.",
            details=[
                "The calibration launcher could not start a basic WSL shell.",
                *(line for line in (wsl_probe.stderr or wsl_probe.stdout).splitlines() if line.strip()),
            ],
        )

    details.append("WSL is reachable.")

    gl_probe_command = [
        sys.executable,
        str(RUN_PIPELINE),
        "bash",
        "bash",
        "-lc",
        "set -e; if [[ -z \"${DISPLAY:-}\" ]]; then echo 'DISPLAY is not set inside the container.' >&2; exit 41; fi; "
        "glxinfo -B",
    ]
    gl_probe = _run(gl_probe_command)

    if gl_probe.returncode != 0:
        output = (gl_probe.stdout or "") + ("\n" + gl_probe.stderr if gl_probe.stderr else "")
        details.append("The calibration container could not complete the OpenGL display probe.")
        details.extend(line for line in output.splitlines() if line.strip())
        return PreflightResult(
            ok=False,
            summary=(
                "Calibration requires a working Docker + WSLg OpenGL path. "
                "The container could not open an accelerated display."
            ),
            details=details,
        )

    output = gl_probe.stdout
    renderer = _extract_value(output, "OpenGL renderer string")
    direct_rendering = _extract_value(output, "direct rendering")

    if renderer:
        details.append(f"OpenGL renderer: {renderer}")
    if direct_rendering:
        details.append(f"Direct rendering: {direct_rendering}")

    if direct_rendering is not None and direct_rendering.lower() != "yes":
        return PreflightResult(
            ok=False,
            summary="Calibration requires direct hardware-backed OpenGL rendering.",
            details=details
            + [
                "The container did not report direct rendering.",
                "Software rendering is unsupported for calibration on Windows.",
            ],
            renderer=renderer,
            direct_rendering=direct_rendering,
        )

    if _looks_like_software_renderer(renderer):
        return PreflightResult(
            ok=False,
            summary="Calibration is using a software OpenGL renderer, which is unsupported.",
            details=details
            + [
                "Software rendering is too slow for the direct visual LiDAR calibration GUI.",
                "Use a machine with a working GPU-backed WSLg OpenGL path.",
            ],
            renderer=renderer,
            direct_rendering=direct_rendering,
        )

    return PreflightResult(
        ok=True,
        summary="Calibration requirements check passed.",
        details=details
        + [
            "Hardware-backed OpenGL was detected inside the calibration container.",
            "This machine meets the minimum graphics requirements for calibration.",
        ],
        renderer=renderer,
        direct_rendering=direct_rendering,
    )


def main() -> int:
    result = run_preflight()
    prefix = "PASS" if result.ok else "FAIL"
    print(f"[preflight] {prefix}: {result.summary}")
    for detail in result.details:
        print(f"[preflight] {detail}")
    return 0 if result.ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
