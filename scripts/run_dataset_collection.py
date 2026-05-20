#!/usr/bin/env python3

"""Dataset collection episode orchestration skeleton.

This script is intentionally kept outside ROS nodes. It is responsible for future
batch simulation orchestration, while DatasetRecorderNode only records data inside
the ROS graph.
"""

import argparse
import subprocess
import sys
from pathlib import Path

try:
    import yaml
except ImportError:  # pragma: no cover - runtime environment guard
    yaml = None


def load_spawn_points(path):
    """Load spawn point configuration from a YAML file."""
    config_path = Path(path)
    if not config_path.exists():
        raise FileNotFoundError(f"spawn points file does not exist: {config_path}")
    if yaml is None:
        raise RuntimeError("PyYAML is required to read spawn point YAML files")

    with config_path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    return data.get("worlds", {})


def choose_spawn_point(spawn_points, seed):
    """Choose one spawn point deterministically from a seed."""
    if not spawn_points:
        return {"x": 0.0, "y": 0.0, "yaw": 0.0}
    return spawn_points[seed % len(spawn_points)]


def build_episode_id(world, episode_index, seed):
    """Build a stable episode id for dataset output directories."""
    return f"{world}_episode_{episode_index:04d}_seed_{seed:04d}"


def run_episode(args, episode_index):
    """Prepare and optionally run one dataset collection episode."""
    seed = episode_index
    worlds = load_spawn_points(args.spawn_points_file)
    world_config = worlds.get(args.world, {})
    spawn_points = world_config.get("spawn_points", [])
    spawn = choose_spawn_point(spawn_points, seed)
    episode_id = build_episode_id(args.world, episode_index, seed)

    print(f"[episode] id={episode_id}")
    print(f"[episode] world={args.world}")
    print(
        "[episode] spawn="
        f"x={spawn.get('x', 0.0)} y={spawn.get('y', 0.0)} yaw={spawn.get('yaw', 0.0)}"
    )
    print(f"[episode] output_dir={args.output_dir}")
    print(f"[episode] timeout_sec={args.timeout_sec}")

    command = [
        "ros2",
        "launch",
        "autonomousr_explorer_bringup",
        "dataset_collection.launch.py",
        f"world:={args.world}",
        f"episode_id:={episode_id}",
        f"dataset_output_dir:={args.output_dir}",
        f"spawn_x:={spawn.get('x', 0.0)}",
        f"spawn_y:={spawn.get('y', 0.0)}",
        f"spawn_yaw:={spawn.get('yaw', 0.0)}",
    ]
    print("[episode] command=" + " ".join(command))

    if args.dry_run:
        return 0

    process = subprocess.Popen(command)
    try:
        return process.wait(timeout=args.timeout_sec)
    except subprocess.TimeoutExpired:
        print(f"[episode] timeout after {args.timeout_sec}s, terminating process")
        process.terminate()
        try:
            return process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            return process.wait()


def parse_args(argv):
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Batch orchestration skeleton for frontier exploration dataset collection."
    )
    parser.add_argument("--world", default="turtlebot3_world")
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--output-dir", default="datasets/frontier_exploration/raw")
    parser.add_argument(
        "--spawn-points-file",
        default="configs/dataset_spawn_points.yaml",
    )
    parser.add_argument("--timeout-sec", type=int, default=600)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args(argv)


def main(argv=None):
    """Run the requested number of dataset collection episodes."""
    args = parse_args(argv or sys.argv[1:])
    for episode_index in range(args.episodes):
        exit_code = run_episode(args, episode_index)
        if exit_code != 0:
            return exit_code
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
