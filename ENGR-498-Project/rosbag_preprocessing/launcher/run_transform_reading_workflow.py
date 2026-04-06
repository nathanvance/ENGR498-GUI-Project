from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


RUN_PIPELINE = Path(__file__).with_name("run_pipeline.py")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run the transform-reading / pose-recovery workflow inside the portable ROS container.",
        epilog=(
            "Example:\n"
            "  python run_transform_reading_workflow.py "
            "<bag_path> --image-topic /camera/image/compressed --gps-topic /fix\n\n"
            "Outputs default to the mounted project folder:\n"
            "  rosbag_preprocessing\\outputs\\pose_recovery"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    _, workflow_args = parser.parse_known_args()

    command = [sys.executable, str(RUN_PIPELINE), "pose-recovery", *workflow_args]
    completed = subprocess.run(command, check=False)
    return completed.returncode


if __name__ == "__main__":
    raise SystemExit(main())
