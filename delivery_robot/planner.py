"""Global route planning: Dijkstra's shortest-path algorithm.

Given the garden-path graph as an adjacency list, this finds the shortest route
from a start node (the supply station) to a goal node (the plant you picked).
The result is the total distance and the list of node ids to drive through.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field


@dataclass
class Route:
    """A planned route: ``dist`` is the total length, ``path`` the node ids.

    If the goal is unreachable, ``dist`` is ``math.inf`` and ``path`` is empty.
    """

    dist: float
    path: list = field(default_factory=list)


def dijkstra(adj, src, dst) -> Route:
    """Shortest path from ``src`` to ``dst`` over the adjacency list ``adj``.

    Keeps a best-known distance to every node, starting at 0 for ``src`` and
    infinity for the rest. It repeatedly settles the nearest unsettled node and
    relaxes every path out of it. A binary heap (``heapq``) supplies the nearest
    unsettled node in O(log n); a ``settled`` array skips the stale heap entries
    that lazy decrease-key leaves behind. Because the closest node is always
    settled first, the distance is final the moment the goal is settled.

    Returns a ``Route``. If ``dst`` is unreachable the distance is ``inf`` and
    the path is empty.
    """
    n = len(adj)
    D = [math.inf] * n          # best known distance from src
    prev = [-1] * n             # previous node on the best route
    settled = [False] * n
    D[src] = 0.0

    pq = [(0.0, src)]           # (distance, node)
    while pq:
        d, u = heapq.heappop(pq)
        if settled[u]:
            continue            # a stale, longer entry for an already-settled node
        settled[u] = True
        if u == dst:
            break               # goal settled: its distance is final

        # Relax every garden path out of u.
        for e in adj[u]:
            nd = d + e.w
            if nd < D[e.to]:
                D[e.to] = nd
                prev[e.to] = u
                heapq.heappush(pq, (nd, e.to))

    if not math.isfinite(D[dst]):
        return Route(dist=math.inf, path=[])

    path = []
    v = dst
    while v != -1:
        path.append(v)
        v = prev[v]
    path.reverse()
    return Route(dist=D[dst], path=path)


def route_waypoints(nodes, path):
    """Node ids -> list of ``(x, y)`` waypoints the robot drives through."""
    return [(nodes[i].x, nodes[i].y) for i in path]
