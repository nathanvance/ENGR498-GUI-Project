from __future__ import annotations

import argparse
import re
import shlex
import subprocess
from pathlib import Path


WINDOWS_PROJECT_ROOT = Path(__file__).resolve().parent.parent
WINDOWS_DRIVE_PATTERN = re.compile(r"^[A-Za-z]:[\\/]")


def to_wsl_path(path: Path) -> str:
    drive = path.drive.rstrip(":").lower()
    tail = path.as_posix().split(":", 1)[-1]
    if drive:
        return f"/mnt/{drive}{tail}"
    return path.as_posix()


def run_wsl(command: str) -> int:
    completed = subprocess.run(["wsl", "bash", "-lc", command], check=False)
    return completed.returncode


def _convert_arg_to_container_path(value: str) -> str:
    if value.startswith("/") or value.startswith("~"):
        return value

    if WINDOWS_DRIVE_PATTERN.match(value):
        return to_wsl_path(Path(value))

    candidate = Path(value)
    if candidate.is_absolute():
        return to_wsl_path(candidate.resolve())

    return to_wsl_path((Path.cwd() / candidate).resolve())


def normalize_workflow_args(mode: str, workflow_args: list[str]) -> list[str]:
    normalized: list[str] = []
    path_like_flags = {
        "calibration": {"--run-root"},
        "pose-recovery": {"--output-root"},
        "bash": set(),
    }
    flags_with_paths = path_like_flags.get(mode, set())

    positional_paths_remaining = 1 if mode in {"calibration", "pose-recovery"} else 0
    next_is_path_value = False

    for arg in workflow_args:
        if next_is_path_value:
            normalized.append(_convert_arg_to_container_path(arg))
            next_is_path_value = False
            continue

        if arg in flags_with_paths:
            normalized.append(arg)
            next_is_path_value = True
            continue

        if positional_paths_remaining > 0 and not arg.startswith("-"):
            normalized.append(_convert_arg_to_container_path(arg))
            positional_paths_remaining -= 1
            continue

        normalized.append(arg)

    return normalized


def derive_windows_root(container_args: list[str]) -> str:
    drives: set[str] = set()
    for arg in container_args:
        match = re.match(r"^/mnt/([A-Za-z])(?:/|$)", arg)
        if match:
            drives.add(match.group(1).lower())

    if not drives:
        return "/mnt/c"
    if len(drives) == 1:
        return f"/mnt/{next(iter(drives))}"
    return "/mnt"


def main() -> int:
    parser = argparse.ArgumentParser(description="Launch the portable ROS Docker workflows through WSL.")
    parser.add_argument("mode", choices=["calibration", "pose-recovery", "bash"])
    parser.add_argument("args", nargs=argparse.REMAINDER, help="Arguments forwarded to the container mode.")
    args = parser.parse_args()

    project_root_wsl = to_wsl_path(WINDOWS_PROJECT_ROOT)
    normalized_args = normalize_workflow_args(args.mode, list(args.args))
    windows_root = derive_windows_root(normalized_args)
    compose_args = " ".join(
        shlex.quote(arg)
        for arg in ["compose", "run", "-T", "--rm", "portable-ros-stack", "bash", "./docker/run_pipeline.sh", args.mode, *normalized_args]
    )
    docker_selector = (
        'if command -v docker >/dev/null 2>&1 && docker version >/dev/null 2>&1; then DOCKER_CMD=docker; '
        'elif command -v docker.exe >/dev/null 2>&1; then DOCKER_CMD=docker.exe; '
        'else echo "ERROR: neither docker nor docker.exe is available." >&2; exit 2; fi'
    )
    docker_cmd = (
        f"cd {shlex.quote(project_root_wsl)} && "
        f"{docker_selector} && "
        f"PORTABLE_ROS_WINDOWS_ROOT={shlex.quote(windows_root)} \"\\$DOCKER_CMD\" {compose_args}"
    )
    return run_wsl(docker_cmd)


if __name__ == "__main__":
    raise SystemExit(main())
