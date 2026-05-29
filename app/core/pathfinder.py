from __future__ import annotations

import heapq
import math
from typing import Dict, List, Tuple

from app.models import MapConfig, NavNode


class PathFinder:
    """
    Dijkstra's shortest-path on the node/edge graph from the map config.
    Edges are bidirectional.  Weight defaults to Euclidean pixel distance
    when not explicitly set in the JSON.
    """

    def __init__(self, map_config: MapConfig) -> None:
        self._nodes: Dict[str, NavNode] = {n.id: n for n in map_config.nodes}
        self._graph: Dict[str, List[Tuple[str, float]]] = self._build(map_config)

    # ── public ────────────────────────────────────────────────────────────────

    def find_path(self, from_id: str, to_id: str) -> List[NavNode]:
        if from_id not in self._graph or to_id not in self._graph:
            return []

        dist: Dict[str, float] = {nid: math.inf for nid in self._nodes}
        prev: Dict[str, str]   = {}
        dist[from_id] = 0.0

        # heap entries: (cost, node_id)
        heap: List[Tuple[float, str]] = [(0.0, from_id)]

        while heap:
            d, u = heapq.heappop(heap)
            if d > dist[u]:
                continue
            if u == to_id:
                break
            for v, w in self._graph[u]:
                nd = dist[u] + w
                if nd < dist[v]:
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(heap, (nd, v))

        if dist[to_id] == math.inf:
            return []

        return self._reconstruct(prev, from_id, to_id)

    # ── private ───────────────────────────────────────────────────────────────

    def _build(self, cfg: MapConfig) -> Dict[str, List[Tuple[str, float]]]:
        graph: Dict[str, List[Tuple[str, float]]] = {n.id: [] for n in cfg.nodes}
        for edge in cfg.edges:
            w = edge.distance if edge.distance is not None else self._euclidean(
                self._nodes[edge.from_node], self._nodes[edge.to]
            )
            graph[edge.from_node].append((edge.to,       w))
            graph[edge.to].append(       (edge.from_node, w))
        return graph

    def _reconstruct(self, prev: Dict[str, str], src: str, dst: str) -> List[NavNode]:
        path: List[str] = []
        cur: str | None = dst
        while cur is not None:
            path.append(cur)
            cur = prev.get(cur)
        path.reverse()
        # Sanity check: path must start at src
        if not path or path[0] != src:
            return []
        return [self._nodes[nid] for nid in path]

    @staticmethod
    def _euclidean(a: NavNode, b: NavNode) -> float:
        return math.hypot(a.x - b.x, a.y - b.y)
