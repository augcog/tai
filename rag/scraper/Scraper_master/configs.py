import yaml
from typing import List
from dataclasses import dataclass
from abc import ABC


@dataclass
class BaseTaskConfig(ABC):
    name: str
    is_local: bool
    root_folder: str
    log_folder: str


@dataclass
class GeneralTaskConfig(BaseTaskConfig):
    start_url: str
    base_url: str
    driver_type: str = "requests"


@dataclass
class RstTaskConfig(BaseTaskConfig):
    # TODO: to be implemented
    pass


class VideoTaskConfig(BaseTaskConfig):
    # TODO: to be implemented
    pass


class ScraperConfig:
    """
    Loads the entire config, including global properties (like 'root_folder')
    and a list of tasks, each represented by a TaskConfig.
    """

    def __init__(self, config_path: str):
        with open(config_path, "r") as f:
            raw = yaml.safe_load(f)

        self.root_folder = raw.get("root_folder", "default_output")
        self.log_folder = raw.get("log_folder", None)
        self.tasks = self._parse_tasks(raw.get("tasks", []))

    def _parse_tasks(self, task_list: list) -> List[BaseTaskConfig]:
        tasks = []
        for t in task_list:
            task_type = t.get("task_type", "html")
            name = t.get("name", "UnnamedTask")

            if task_type == "html":
                # Extract fields specifically for HtmlTaskConfig
                obj = GeneralTaskConfig(
                    name=name,
                    is_local=t.get("local", False),
                    start_url=t.get("url", ""),
                    base_url=t.get("root", ""),
                    root_folder=self.root_folder,
                    driver_type=t.get("driver_type", "requests"),
                    log_folder=self.log_folder,
                )
                tasks.append(obj)

            elif task_type == "rst":
                # TODO implement RstTaskConfig
                pass

            else:
                # Potentially throw an error or have a default
                raise ValueError(f"Unrecognized task type: {task_type}")
        return tasks
