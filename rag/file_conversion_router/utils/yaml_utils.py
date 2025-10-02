"""YAML file utilities for loading and saving configuration files."""

import yaml
from typing import Any, Dict


def load_yaml(file_path: str) -> Dict[str, Any]:
    """Load YAML configuration file.

    Args:
        file_path: Path to the YAML file to load

    Returns:
        Dictionary containing the loaded YAML data
    """
    with open(file_path, "r") as file:
        return yaml.safe_load(file)


def save_yaml(data: Dict[str, Any], file_path: str) -> None:
    """Save data to YAML configuration file.

    Args:
        data: Dictionary data to save
        file_path: Path where the YAML file will be saved
    """
    with open(file_path, "w") as file:
        yaml.safe_dump(data, file, default_flow_style=False, indent=2)