from pathlib import Path
from typing import Optional

import yaml


def resolve_map_file(bringup_pkg: str,
                     config_filename: str = "explore_map.yaml",
                     default_map_dir: str = "maps",
                     default_map_name: str = "explore_map.yaml") -> str:
    """
    Determine the map file to load using two config parameters:
    - map_directory: absolute/relative path to the directory containing map files.
    - map_name: the filename within that directory.

    When map_directory is empty, fall back to a relative workspace lookup that lands in
    the top-level `maps/` folder next to the workspace root.
    """
    pkg_path = Path(bringup_pkg)
    config_path = pkg_path / "config" / config_filename
    config_data = _load_yaml(config_path)

    maps_dir = _resolve_map_directory(config_path, config_data, pkg_path, default_map_dir)
    map_name = _extract_map_name(config_data, default_map_name)

    return str((maps_dir / map_name).resolve())


def _load_yaml(config_path: Path) -> dict:
    if not config_path.is_file():
        return {}

    try:
        return yaml.safe_load(config_path.read_text()) or {}
    except Exception:
        return {}


def _resolve_map_directory(config_path: Path,
                           config_data: dict,
                           pkg_path: Path,
                           default_map_dir: str) -> Path:
    candidate = _extract_map_directory(config_data)
    if candidate:
        candidate_path = Path(candidate).expanduser()
        if not candidate_path.is_absolute():
            candidate_path = (config_path.parent / candidate_path).resolve()
        return candidate_path

    return _default_maps_dir(pkg_path, default_map_dir)


def _extract_map_directory(config_data: dict) -> Optional[str]:
    if not isinstance(config_data, dict):
        return None

    for key in ("map_directory", "maps_directory", "map_dir"):
        value = config_data.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip()

    return None


def _default_maps_dir(pkg_path: Path, default_map_dir: str) -> Path:
    # Try to locate the workspace root (`.../install` -> workspace) and append maps/
    parents = list(pkg_path.parents)
    workspace_root: Optional[Path] = None
    if len(parents) >= 3:
        workspace_root = parents[3]

    if workspace_root is None:
        return (pkg_path / default_map_dir).resolve()

    return (workspace_root / default_map_dir).resolve()


def _extract_map_name(config_data: dict, default_name: str) -> str:
    if not isinstance(config_data, dict):
        return default_name

    value = config_data.get("map_name")
    if isinstance(value, str) and value.strip():
        return value.strip()

    return default_name
