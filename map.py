# map_graph.py  — offline planner side

from enum import IntEnum
from typing import NamedTuple, List


# ------------ headings / turns (for later) ------------

class Heading(IntEnum):
    N = 0
    E = 1
    S = 2
    W = 3


class Turn(IntEnum):
    F = 0   # straight
    L = 1   # left
    R = 2   # right
    B = 3   # 180° (if you ever want it)


# ------------ vertices ------------

class V(IntEnum):
    DOWN_LEFT   = 0
    A_DOWN_BEG  = 1
    A_DOWN_END  = 2
    BLUE        = 3
    LEFT        = 4
    GREEN       = 5
    START_LEFT  = 6
    START       = 7
    START_RIGHT = 8
    YELLOW      = 9
    RIGHT       = 10
    RED         = 11
    B_DOWN_BEG  = 12
    B_DOWN_END  = 13
    DOWN_RIGHT  = 14
    A_UP_BEG    = 15
    A_UP_END    = 16
    UP_LEFT     = 17
    RAMP        = 18
    UP_RIGHT    = 19
    B_UP_BEG    = 20
    B_UP_END    = 21


VERTEX_NAMES = {
    V.DOWN_LEFT:   "DOWN_LEFT",
    V.A_DOWN_BEG:  "A_DOWN_BEG",
    V.A_DOWN_END:  "A_DOWN_END",
    V.BLUE:        "BLUE",
    V.LEFT:        "LEFT",
    V.GREEN:       "GREEN",
    V.START_LEFT:  "START_LEFT",
    V.START:       "START",
    V.START_RIGHT: "START_RIGHT",
    V.YELLOW:      "YELLOW",
    V.RIGHT:       "RIGHT",
    V.RED:         "RED",
    V.B_DOWN_BEG:  "B_DOWN_BEG",
    V.B_DOWN_END:  "B_DOWN_END",
    V.DOWN_RIGHT:  "DOWN_RIGHT",
    V.A_UP_BEG:    "A_UP_BEG",
    V.A_UP_END:    "A_UP_END",
    V.UP_LEFT:     "UP_LEFT",
    V.RAMP:        "RAMP",
    V.UP_RIGHT:    "UP_RIGHT",
    V.B_UP_BEG:    "B_UP_BEG",
    V.B_UP_END:    "B_UP_END",
}
class Segment(NamedTuple):
    u: V
    v: V
    length: float  # placeholder; tune from real geometry


class DirectedEdge(NamedTuple):
    src: V
    dst: V
    start_heading: Heading
    turn: Turn
    end_heading: Heading
    cost: float



DIRECTED_EDGES = [

    DirectedEdge(src=V.DOWN_LEFT  , dst=V.DOWN_RIGHT , start_heading=1, turn=None, end_heading=1, cost=8.0),
    DirectedEdge(src=V.DOWN_LEFT  , dst=V.RAMP       , start_heading=1, turn=None, end_heading=2, cost=8.0),
    DirectedEdge(src=V.DOWN_LEFT  , dst=V.A_DOWN_BEG , start_heading=3, turn=None, end_heading=2, cost=6.0),

    DirectedEdge(src=V.A_DOWN_BEG , dst=V.DOWN_LEFT  , start_heading=0, turn=None, end_heading=1, cost=6.0),
    DirectedEdge(src=V.A_DOWN_BEG , dst=V.A_DOWN_END , start_heading=2, turn=None, end_heading=2, cost=6.0),

    DirectedEdge(src=V.A_DOWN_END , dst=V.A_DOWN_BEG , start_heading=0, turn=None, end_heading=0, cost=6.0),
    DirectedEdge(src=V.A_DOWN_END , dst=V.BLUE       , start_heading=2, turn=None, end_heading=2, cost=6.0),
    DirectedEdge(src=V.A_DOWN_END , dst=V.LEFT       , start_heading=2, turn=None, end_heading=1, cost=5.0),

    DirectedEdge(src=V.BLUE       , dst=V.A_DOWN_END , start_heading=0, turn=None, end_heading=0, cost=6.0),
    DirectedEdge(src=V.BLUE       , dst=V.LEFT       , start_heading=0, turn=None, end_heading=1, cost=3.0),

    DirectedEdge(src=V.LEFT       , dst=V.A_DOWN_END , start_heading=3, turn=None, end_heading=0, cost=5.0),
    DirectedEdge(src=V.LEFT       , dst=V.BLUE       , start_heading=3, turn=None, end_heading=2, cost=3.0),
    DirectedEdge(src=V.LEFT       , dst=V.GREEN      , start_heading=1, turn=None, end_heading=2, cost=3.0),
    DirectedEdge(src=V.LEFT       , dst=V.START_LEFT , start_heading=1, turn=None, end_heading=1, cost=3.0),

    DirectedEdge(src=V.GREEN      , dst=V.LEFT       , start_heading=0, turn=None, end_heading=3, cost=3.0),
    DirectedEdge(src=V.GREEN      , dst=V.START_LEFT , start_heading=0, turn=None, end_heading=1, cost=4.0),

    DirectedEdge(src=V.START_LEFT , dst=V.LEFT       , start_heading=3, turn=None, end_heading=3, cost=3.0),
    DirectedEdge(src=V.START_LEFT , dst=V.GREEN      , start_heading=3, turn=None, end_heading=2, cost=4.0),
    DirectedEdge(src=V.START_LEFT , dst=V.START      , start_heading=1, turn=None, end_heading=2, cost=4.0),
    DirectedEdge(src=V.START_LEFT , dst=V.START_RIGHT, start_heading=1, turn=None, end_heading=1, cost=4.0),

    DirectedEdge(src=V.START      , dst=V.START_LEFT , start_heading=0, turn=None, end_heading=3, cost=4.0),
    DirectedEdge(src=V.START      , dst=V.START_RIGHT, start_heading=0, turn=None, end_heading=1, cost=4.0),

    DirectedEdge(src=V.START_RIGHT, dst=V.START      , start_heading=3, turn=None, end_heading=3, cost=4.0),
    DirectedEdge(src=V.START_RIGHT, dst=V.START_LEFT , start_heading=3, turn=None, end_heading=2, cost=4.0),
    DirectedEdge(src=V.START_RIGHT, dst=V.RIGHT      , start_heading=1, turn=None, end_heading=1, cost=3.0),
    DirectedEdge(src=V.START_RIGHT, dst=V.YELLOW     , start_heading=1, turn=None, end_heading=2, cost=4.0),

    DirectedEdge(src=V.YELLOW     , dst=V.START_RIGHT, start_heading=0, turn=None, end_heading=3, cost=4.0),
    DirectedEdge(src=V.YELLOW     , dst=V.RIGHT      , start_heading=0, turn=None, end_heading=1, cost=3.0),

    DirectedEdge(src=V.RIGHT      , dst=V.START_RIGHT, start_heading=3, turn=None, end_heading=3, cost=3.0),
    DirectedEdge(src=V.RIGHT      , dst=V.YELLOW     , start_heading=3, turn=None, end_heading=2, cost=3.0),
    DirectedEdge(src=V.RIGHT      , dst=V.B_DOWN_BEG , start_heading=1, turn=None, end_heading=0, cost=5.0),
    DirectedEdge(src=V.RIGHT      , dst=V.RED        , start_heading=1, turn=None, end_heading=2, cost=3.0),

    DirectedEdge(src=V.RED        , dst=V.RIGHT      , start_heading=0, turn=None, end_heading=3, cost=3.0),
    DirectedEdge(src=V.RED        , dst=V.B_DOWN_BEG , start_heading=0, turn=None, end_heading=0, cost=6.0),

    DirectedEdge(src=V.B_DOWN_BEG , dst=V.RIGHT      , start_heading=2, turn=None, end_heading=3, cost=5.0),
    DirectedEdge(src=V.B_DOWN_BEG , dst=V.RED        , start_heading=2, turn=None, end_heading=2, cost=6.0),
    DirectedEdge(src=V.B_DOWN_BEG , dst=V.B_DOWN_END , start_heading=0, turn=None, end_heading=0, cost=6.0),

    DirectedEdge(src=V.B_DOWN_END , dst=V.B_DOWN_BEG , start_heading=2, turn=None, end_heading=2, cost=6.0),
    DirectedEdge(src=V.B_DOWN_END , dst=V.DOWN_RIGHT , start_heading=0, turn=None, end_heading=3, cost=6.0),

    DirectedEdge(src=V.DOWN_RIGHT , dst=V.B_DOWN_END , start_heading=1, turn=None, end_heading=2, cost=6.0),
    DirectedEdge(src=V.DOWN_RIGHT , dst=V.DOWN_LEFT  , start_heading=3, turn=None, end_heading=2, cost=8.0),
    DirectedEdge(src=V.DOWN_RIGHT , dst=V.RAMP       , start_heading=3, turn=None, end_heading=3, cost=8.0),

    DirectedEdge(src=V.A_UP_END   , dst=V.UP_LEFT    , start_heading=0, turn=None, end_heading=1, cost=3.0),
    DirectedEdge(src=V.A_UP_END   , dst=V.B_UP_BEG   , start_heading=2, turn=None, end_heading=0, cost=6.0),

    DirectedEdge(src=V.A_UP_BEG   , dst=V.A_UP_END   , start_heading=2, turn=None, end_heading=2, cost=6.0),

    DirectedEdge(src=V.UP_LEFT    , dst=V.A_UP_END   , start_heading=3, turn=None, end_heading=0, cost=3.0),
    DirectedEdge(src=V.UP_LEFT    , dst=V.RAMP       , start_heading=1, turn=None, end_heading=0, cost=5.0),
    DirectedEdge(src=V.UP_LEFT    , dst=V.UP_RIGHT   , start_heading=1, turn=None, end_heading=1, cost=2.0),

    DirectedEdge(src=V.RAMP       , dst=V.UP_LEFT    , start_heading=2, turn=None, end_heading=3, cost=5.0),
    DirectedEdge(src=V.RAMP       , dst=V.UP_RIGHT   , start_heading=2, turn=None, end_heading=1, cost=5.0),
    DirectedEdge(src=V.RAMP       , dst=V.DOWN_LEFT  , start_heading=0, turn=None, end_heading=3, cost=8.0),
    DirectedEdge(src=V.RAMP       , dst=V.DOWN_RIGHT , start_heading=0, turn=None, end_heading=1, cost=8.0),

    DirectedEdge(src=V.UP_RIGHT   , dst=V.B_UP_BEG   , start_heading=1, turn=None, end_heading=0, cost=3.0),
    DirectedEdge(src=V.UP_RIGHT   , dst=V.RAMP       , start_heading=3, turn=None, end_heading=0, cost=5.0),
    DirectedEdge(src=V.UP_RIGHT   , dst=V.UP_LEFT    , start_heading=3, turn=None, end_heading=3, cost=2.0),

    DirectedEdge(src=V.B_UP_BEG   , dst=V.UP_RIGHT   , start_heading=2, turn=None, end_heading=3, cost=3.0),
    DirectedEdge(src=V.B_UP_BEG   , dst=V.B_UP_END   , start_heading=0, turn=None, end_heading=0, cost=6.0),

    DirectedEdge(src=V.B_UP_END   , dst=V.B_UP_BEG   , start_heading=2, turn=None, end_heading=2, cost=6.0),

]





# integers for headings
N, E, S, W = 0, 1, 2, 3
F, L, R, B = 0, 1, 2, 3

# vertex IDs are already 0..17 from V enum

SEGMENTS_INT = [
    # u, v, length
    (V.DOWN_LEFT,  V.DOWN_RIGHT,  1),
    (V.DOWN_LEFT,  V.RAMP,        1),
    # ...
]

DIRECTED_EDGES_INT = [
    # src, dst, start_heading, turn, end_heading, cost
    (V.START, V.LEFT,  W, F, W, 1),
    # ...
]
