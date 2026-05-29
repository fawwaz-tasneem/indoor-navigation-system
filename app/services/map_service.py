import json
from pathlib import Path

from app.models import MapConfig

_MAPS_DIR = Path(__file__).parent.parent.parent / "maps"


class MapService:
    def __init__(self, map_file: str = "sample_map.json") -> None:
        self._config = self._load(map_file)

    def _load(self, filename: str) -> MapConfig:
        path = _MAPS_DIR / filename
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        return MapConfig.model_validate(data)

    def get_config(self) -> MapConfig:
        return self._config
