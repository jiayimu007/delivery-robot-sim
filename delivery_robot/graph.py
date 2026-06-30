"""Weighted path graph for the garden-tending robot simulation.

The garden paths are a weighted graph: junctions are nodes, path segments are
edges, and each edge weight is the length of that segment. This module holds the
geometry helpers and turns a node/edge list into an adjacency list that the
planner consumes.

This is a software model written to study the idea. It is not firmware for a
real robot.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Node:
    """A junction (and, when ``place`` is set, a named plant or the station)."""

    id: int
    x: float
    y: float
    place: str | None = None


@dataclass(frozen=True)
class Edge:
    """A directed half of a two-way garden path, used inside the adjacency list."""

    to: int
    w: float


def dist(p, q) -> float:
    """Euclidean distance between two points that expose ``.x`` and ``.y``."""
    return math.hypot(p.x - q.x, p.y - q.y)


def build_adj(nodes, edges):
    """Adjacency list ``adj[u] = [Edge(to, w), ...]`` with weight = path length.

    ``edges`` is a list of ``(a, b)`` node-id pairs. Each garden path is two-way,
    so it is stored in both directions with the same weight, which keeps the
    graph undirected.
    """
    adj = [[] for _ in nodes]
    for a, b in edges:
        w = dist(nodes[a], nodes[b])
        adj[a].append(Edge(to=b, w=w))
        adj[b].append(Edge(to=a, w=w))
    return adj
