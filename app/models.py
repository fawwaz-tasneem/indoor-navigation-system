from __future__ import annotations

from enum import Enum
from typing import List, Optional

from pydantic import BaseModel, ConfigDict, Field
from pydantic.alias_generators import to_camel


class _CamelModel(BaseModel):
    """Base model: accepts camelCase JSON in, serialises camelCase JSON out."""
    model_config = ConfigDict(alias_generator=to_camel, populate_by_name=True)


class NodeType(str, Enum):
    CORRIDOR  = "CORRIDOR"
    CLASSROOM = "CLASSROOM"
    OFFICE    = "OFFICE"
    LAB       = "LAB"
    SEMINAR   = "SEMINAR"
    LIBRARY   = "LIBRARY"
    ENTRANCE  = "ENTRANCE"
    TOILET    = "TOILET"
    STAIRCASE = "STAIRCASE"


class NavNode(_CamelModel):
    id:    str
    label: str
    type:  NodeType
    x:     float
    y:     float


class NavEdge(_CamelModel):
    # "from" is a Python keyword — alias keeps JSON key intact
    from_node: str  = Field(alias="from")
    to:        str
    distance:  Optional[float] = None


class AccessPoint(_CamelModel):
    id:                  str
    ssid:                str
    bssid:               str
    x:                   float
    y:                   float
    tx_power:            float = -40.0
    path_loss_exponent:  float = 2.7


class MapConfig(_CamelModel):
    building_name:    str
    canvas_width:     int
    canvas_height:    int
    meters_per_pixel: float = 0.05
    nodes:            List[NavNode]
    edges:            List[NavEdge]
    access_points:    List[AccessPoint]


class Position(_CamelModel):
    x:           float
    y:           float
    uncertainty: float
