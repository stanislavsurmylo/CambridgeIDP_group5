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
    A_DOWN      = 1
    BLUE        = 2
    LEFT        = 3
    GREEN       = 4
    START_LEFT  = 5
    START       = 6
    START_RIGHT = 7
    YELLOW      = 8
    RIGHT       = 9
    RED         = 10
    B_DOWN      = 11
    DOWN_RIGHT  = 12
    A_UP        = 13
    UP_LEFT     = 14
    RAMP        = 15
    UP_RIGHT    = 16
    B_UP        = 17


VERTEX_NAMES = {
    V.DOWN_LEFT:   "DOWN_LEFT",
    V.A_DOWN:      "A_DOWN",
    V.BLUE:        "BLUE",
    V.LEFT:        "LEFT",
    V.GREEN:       "GREEN",
    V.START_LEFT:  "START_LEFT",
    V.START:       "START",
    V.START_RIGHT: "START_RIGHT",
    V.YELLOW:      "YELLOW",
    V.RIGHT:       "RIGHT",
    V.RED:         "RED",
    V.B_DOWN:      "B_DOWN",
    V.DOWN_RIGHT:  "DOWN_RIGHT",
    V.A_UP:        "A_UP",
    V.UP_LEFT:     "UP_LEFT",
    V.RAMP:        "RAMP",
    V.UP_RIGHT:    "UP_RIGHT",
    V.B_UP:        "B_UP",
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



# List of undirected segments (each physical red line in your sketch)
DIRECTEDEDGES: List[Segment] = [
    # top + ramp connections
    Segment(V.DOWN_LEFT,  V.DOWN_RIGHT,  1.0),
    Segment(V.DOWN_LEFT,  V.RAMP,        1.0),
    Segment(V.DOWN_RIGHT, V.RAMP,        1.0),

    # left vertical + blue triangle
    Segment(V.DOWN_LEFT,  V.A_DOWN,      1.0),
    Segment(V.A_DOWN,     V.BLUE,        1.0),
    Segment(V.A_DOWN,     V.LEFT,        1.0),
    Segment(V.BLUE,       V.LEFT,        1.0),

    # bottom middle chain (outer lane)
    Segment(V.LEFT,       V.START_LEFT,  1.0),
    Segment(V.START_LEFT, V.START_RIGHT, 1.0),
    Segment(V.START_LEFT, V.START,       1.0),
    Segment(V.START_RIGHT,V.START,       1.0),
    Segment(V.START_RIGHT,V.RIGHT,       1.0),

    # colour bays
    Segment(V.START_LEFT, V.GREEN,       1.0),
    Segment(V.START_RIGHT,V.YELLOW,      1.0),
    Segment(V.LEFT,       V.GREEN,       1.0),   # that left–green diagonal
    Segment(V.START_RIGHT,V.YELLOW,      1.0),
    Segment(V.RIGHT,      V.YELLOW,      1.0),
    Segment(V.BLUE,       V.A_DOWN,      1.0),
    Segment(V.RED,        V.B_DOWN,      1.0),

    # right bottom + red triangle
    Segment(V.RIGHT,      V.B_DOWN,      1.0),
    Segment(V.RIGHT,      V.RED,         1.0),
    Segment(V.B_DOWN,     V.DOWN_RIGHT,  1.0),

    # upper inner loop + ramp
    Segment(V.A_UP,       V.UP_LEFT,     1.0),
    Segment(V.B_UP,       V.UP_RIGHT,    1.0),
    Segment(V.UP_LEFT,    V.UP_RIGHT,    1.0),
    Segment(V.UP_LEFT,    V.RAMP,        1.0),
    Segment(V.UP_RIGHT,   V.RAMP,        1.0),
]



# Example macro edges — you will fill these properly
DIRECTED_EDGES: List[DirectedEdge] = [
    # Example: from START to LEFT along the bottom lane, starting facing West
    # (fill headings/turns according to how you define N/E/S/W on the real board)
    DirectedEdge(
        src=V.START,
        dst=V.LEFT,
        start_heading=Heading.W,
        turn=Turn.F,         # just go straight to the left junction
        end_heading=Heading.W,
        cost=1.0,
    ),

    # Example: from START to GREEN via START_LEFT with one junction decision
    # (just to show shape – actual heading/turn may differ)
    DirectedEdge(
        src=V.START,
        dst=V.GREEN,
        start_heading=Heading.S,
        turn=Turn.L,         # “turn left at the first branch” style macro
        end_heading=Heading.S,
        cost=2.0,            # two segments worth, etc.
    ),

    # ...add all the other macros you want
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
