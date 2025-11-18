from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Dict, List


class Alignment(Enum):
    HORIZONTAL = auto()
    VERTICAL = auto()


class Direction(Enum):
    NORTH = auto()
    EAST  = auto()
    SOUTH = auto()
    WEST  = auto()


class Rotation(Enum):
    L     = auto()   # -90°
    R    = auto()   # +90°
    S = auto()   # 0°
    B    = auto()   # 180°


@dataclass
class Edge:
    src: str
    dst: str
    approach_dir: Direction
    rotation: Rotation


@dataclass
class Vertex:
    id: str
    alignment: Alignment
    edges: List[Edge] = field(default_factory=list)


Graph = Dict[str, Vertex]
