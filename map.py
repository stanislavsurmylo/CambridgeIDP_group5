# map_graph.py  — offline planner side

try:
    from typing import Dict, List, Tuple, Optional
    AdjList = Dict["V", List[Tuple["V", float]]]
except ImportError:
    Dict = dict
    List = list
    Tuple = tuple
    Optional = object
    AdjList = dict  # just a runtime alias; hints don’t matter on Pico
# --- defaultdict fallback ---------------------------------------
# --- IntEnum fallback -------------------------------------------------
try:
    from enum import IntEnum
except ImportError:
    # Minimal stand-in for MicroPython
    class IntEnum(int):
        
        def __new__(cls, value):
            return int.__new__(cls, value)

import heapq


# ------------ headings / turns (for later) ------------


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

class DirectedEdge:
    __slots__ = ("src", "dst", "start_heading", "turn", "end_heading", "cost")

    def __init__(self, src: V, dst: V,
        start_heading: int, turn: str,
        end_heading: int, cost: float):
        self.src = src
        self.dst = dst
        self.start_heading = start_heading
        self.turn = turn
        self.end_heading = end_heading
        self.cost = cost


# ------------ directed edges ------------


DIRECTED_EDGES = [

    DirectedEdge(src=V.DOWN_LEFT  , dst=V.DOWN_RIGHT , start_heading=1, turn='F', end_heading=1, cost=8.0),
    DirectedEdge(src=V.DOWN_LEFT  , dst=V.RAMP       , start_heading=1, turn='R', end_heading=2, cost=8.0),
    DirectedEdge(src=V.DOWN_LEFT  , dst=V.A_DOWN_BEG , start_heading=3, turn='L', end_heading=2, cost=6.0),

    DirectedEdge(src=V.A_DOWN_BEG , dst=V.DOWN_LEFT  , start_heading=0, turn='R', end_heading=1, cost=6.0),
    DirectedEdge(src=V.A_DOWN_BEG , dst=V.A_DOWN_END , start_heading=2, turn='F', end_heading=2, cost=6.0),

    DirectedEdge(src=V.A_DOWN_END , dst=V.A_DOWN_BEG , start_heading=0, turn='F', end_heading=0, cost=6.0),
    DirectedEdge(src=V.A_DOWN_END , dst=V.BLUE       , start_heading=2, turn='F', end_heading=2, cost=6.0),
    DirectedEdge(src=V.A_DOWN_END , dst=V.LEFT       , start_heading=2, turn='L', end_heading=1, cost=5.0),

    DirectedEdge(src=V.BLUE       , dst=V.A_DOWN_END , start_heading=0, turn='F', end_heading=0, cost=6.0),
    DirectedEdge(src=V.BLUE       , dst=V.LEFT       , start_heading=0, turn='R', end_heading=1, cost=3.0),

    DirectedEdge(src=V.LEFT       , dst=V.A_DOWN_END , start_heading=3, turn='R', end_heading=0, cost=5.0),
    DirectedEdge(src=V.LEFT       , dst=V.BLUE       , start_heading=3, turn='L', end_heading=2, cost=3.0),
    DirectedEdge(src=V.LEFT       , dst=V.GREEN      , start_heading=1, turn='R', end_heading=2, cost=3.0),
    DirectedEdge(src=V.LEFT       , dst=V.START_LEFT , start_heading=1, turn='F', end_heading=1, cost=3.0),

    DirectedEdge(src=V.GREEN      , dst=V.LEFT       , start_heading=0, turn='L', end_heading=3, cost=3.0),
    DirectedEdge(src=V.GREEN      , dst=V.START_LEFT , start_heading=0, turn='R', end_heading=1, cost=4.0),

    DirectedEdge(src=V.START_LEFT , dst=V.LEFT       , start_heading=3, turn='F', end_heading=3, cost=3.0),
    DirectedEdge(src=V.START_LEFT , dst=V.GREEN      , start_heading=3, turn='L', end_heading=2, cost=4.0),
    DirectedEdge(src=V.START_LEFT , dst=V.START      , start_heading=1, turn='R', end_heading=2, cost=4.0),
    DirectedEdge(src=V.START_LEFT , dst=V.START_RIGHT, start_heading=1, turn='F', end_heading=1, cost=4.0),

    DirectedEdge(src=V.START      , dst=V.START_LEFT , start_heading=0, turn='L', end_heading=3, cost=4.0),
    DirectedEdge(src=V.START      , dst=V.START_RIGHT, start_heading=0, turn='R', end_heading=1, cost=4.0),

    DirectedEdge(src=V.START_RIGHT, dst=V.START      , start_heading=3, turn='L', end_heading=2, cost=4.0),
    DirectedEdge(src=V.START_RIGHT, dst=V.START_LEFT , start_heading=3, turn='F', end_heading=3, cost=4.0),
    DirectedEdge(src=V.START_RIGHT, dst=V.RIGHT      , start_heading=1, turn='F', end_heading=1, cost=3.0),
    DirectedEdge(src=V.START_RIGHT, dst=V.YELLOW     , start_heading=1, turn='R', end_heading=2, cost=4.0),

    DirectedEdge(src=V.YELLOW     , dst=V.START_RIGHT, start_heading=0, turn='L', end_heading=3, cost=4.0),
    DirectedEdge(src=V.YELLOW     , dst=V.RIGHT      , start_heading=0, turn='R', end_heading=1, cost=3.0),

    DirectedEdge(src=V.RIGHT      , dst=V.START_RIGHT, start_heading=3, turn='F', end_heading=3, cost=3.0),
    DirectedEdge(src=V.RIGHT      , dst=V.YELLOW     , start_heading=3, turn='L', end_heading=2, cost=3.0),
    DirectedEdge(src=V.RIGHT      , dst=V.B_DOWN_BEG , start_heading=1, turn='L', end_heading=0, cost=5.0),
    DirectedEdge(src=V.RIGHT      , dst=V.RED        , start_heading=1, turn='R', end_heading=2, cost=3.0),

    DirectedEdge(src=V.RED        , dst=V.RIGHT      , start_heading=0, turn='L', end_heading=3, cost=3.0),
    DirectedEdge(src=V.RED        , dst=V.B_DOWN_BEG , start_heading=0, turn='F', end_heading=0, cost=6.0),

    DirectedEdge(src=V.B_DOWN_BEG , dst=V.RIGHT      , start_heading=2, turn='R', end_heading=3, cost=5.0),
    DirectedEdge(src=V.B_DOWN_BEG , dst=V.RED        , start_heading=2, turn='F', end_heading=2, cost=6.0),
    DirectedEdge(src=V.B_DOWN_BEG , dst=V.B_DOWN_END , start_heading=0, turn='F', end_heading=0, cost=6.0),

    DirectedEdge(src=V.B_DOWN_END , dst=V.B_DOWN_BEG , start_heading=2, turn='F', end_heading=2, cost=6.0),
    DirectedEdge(src=V.B_DOWN_END , dst=V.DOWN_RIGHT , start_heading=0, turn='L', end_heading=3, cost=6.0),

    DirectedEdge(src=V.DOWN_RIGHT , dst=V.B_DOWN_END , start_heading=1, turn='R', end_heading=2, cost=6.0),
    DirectedEdge(src=V.DOWN_RIGHT , dst=V.DOWN_LEFT  , start_heading=3, turn='L', end_heading=2, cost=8.0),
    DirectedEdge(src=V.DOWN_RIGHT , dst=V.RAMP       , start_heading=3, turn='F', end_heading=3, cost=8.0),

    DirectedEdge(src=V.A_UP_BEG   , dst=V.UP_LEFT    , start_heading=0, turn='R', end_heading=1, cost=3.0),
    DirectedEdge(src=V.A_UP_BEG   , dst=V.A_UP_END   , start_heading=0, turn='F', end_heading=0, cost=6.0), 
    
    DirectedEdge(src=V.A_UP_END   , dst=V.A_UP_BEG   , start_heading=2, turn='F', end_heading=2, cost=6.0),

    DirectedEdge(src=V.UP_LEFT    , dst=V.A_UP_BEG   , start_heading=3, turn='R', end_heading=0, cost=3.0),
    DirectedEdge(src=V.UP_LEFT    , dst=V.RAMP       , start_heading=1, turn='L', end_heading=0, cost=5.0),
    DirectedEdge(src=V.UP_LEFT    , dst=V.UP_RIGHT   , start_heading=1, turn='F', end_heading=1, cost=2.0),

    DirectedEdge(src=V.RAMP       , dst=V.UP_LEFT    , start_heading=2, turn='R', end_heading=3, cost=5.0),
    DirectedEdge(src=V.RAMP       , dst=V.UP_RIGHT   , start_heading=2, turn='L', end_heading=1, cost=5.0),
    DirectedEdge(src=V.RAMP       , dst=V.DOWN_LEFT  , start_heading=0, turn='L', end_heading=3, cost=8.0),
    DirectedEdge(src=V.RAMP       , dst=V.DOWN_RIGHT , start_heading=0, turn='R', end_heading=1, cost=8.0),

    DirectedEdge(src=V.UP_RIGHT   , dst=V.B_UP_BEG   , start_heading=1, turn='L', end_heading=0, cost=3.0),
    DirectedEdge(src=V.UP_RIGHT   , dst=V.RAMP       , start_heading=3, turn='R', end_heading=0, cost=5.0),
    DirectedEdge(src=V.UP_RIGHT   , dst=V.UP_LEFT    , start_heading=3, turn='F', end_heading=3, cost=2.0),

    DirectedEdge(src=V.B_UP_BEG   , dst=V.UP_RIGHT   , start_heading=2, turn='R', end_heading=3, cost=3.0),
    DirectedEdge(src=V.B_UP_BEG   , dst=V.B_UP_END   , start_heading=0, turn='F', end_heading=0, cost=6.0),

    DirectedEdge(src=V.B_UP_END   , dst=V.B_UP_BEG   , start_heading=2, turn='F', end_heading=2, cost=6.0),

]







def build_graph(edges: List[DirectedEdge]) -> AdjList:
    # plain dict is enough; no need for defaultdict
    graph = {}  # type: ignore[assignment]  # if mypy complains

    for e in edges:
        # ensure src list exists, then append
        graph.setdefault(e.src, []).append((e.dst, e.cost))
        # ensure dst also exists as a key (even if it has no outgoing edges)
        graph.setdefault(e.dst, [])

    return graph
GRAPH: AdjList = build_graph(DIRECTED_EDGES)

# ------------------------------------------------------------
# Dijkstra from any START to all vertices
# ------------------------------------------------------------

def dijkstra(graph: AdjList, start: "V") -> Tuple[Dict["V", float], Dict["V", Optional["V"]]]:
    dist: Dict["V", float] = {v: 100 for v in graph.keys()}
    prev: Dict["V", Optional["V"]] = {v: None for v in graph.keys()}

    dist[start] = 0.0
    heap: List[Tuple[float, "V"]] = [(0.0, start)]  # (distance, vertex)

    while heap:
        d_u, u = heapq.heappop(heap)




        # Skip stale entries
        if d_u != dist[u]:
            continue

        for v, w in graph[u]:
            nd = d_u + w
            if nd < dist[v]:
                dist[v] = nd
                prev[v] = u
                heapq.heappush(heap, (nd, v))

    return dist, prev

# ------------------------------------------------------------
# Recover the path from START to FINISH
# ------------------------------------------------------------

def shortest_path(graph: AdjList, start: "V", finish: "V") -> List["V"]:
    dist, prev = dijkstra(graph, start)

    if dist[finish] == float("inf"):
        # No route
        return []

    path: List["V"] = []
    v: Optional["V"] = finish
    while v is not None:
        path.append(v)
        v = prev[v]
    path.reverse()
    return path





# ------------------------------------------------------------
# Example usage (assuming V.START and V.BLUE exist)
# ------------------------------------------------------------
if __name__ == "__main__":
     start = V.B_UP_END         # any START vertex
     finish = V.GREEN          # any FINISH vertex

     path = shortest_path(GRAPH, start, finish)
     print("Shortest path:", path)  