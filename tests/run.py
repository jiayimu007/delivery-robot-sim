"""Headless tests for the delivery-robot simulation.

    python tests/run.py
    # or, if pytest is installed:
    pytest tests/run.py

Two things are checked:
  - the planner: Dijkstra's distances and paths are compared against an
    independent Floyd-Warshall oracle, on the real map and on hundreds of
    seeded random graphs (including disconnected ones), so a wrong shortest path
    cannot slip through;
  - the driver: the robot actually follows each planned route to the goal,
    staying close to the road, and the steering/kinematics turn the right way.

The whole file runs as a script and exits non-zero on any failure. The same
checks are also exposed as ``test_*`` functions so pytest can collect them.
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from delivery_robot import town                                    # noqa: E402
from delivery_robot.controller import (                            # noqa: E402
    ARRIVE_RADIUS,
    Car,
    drive_route,
    step_kinematics,
    steer_toward,
)
from delivery_robot.graph import build_adj, dist                   # noqa: E402
from delivery_robot.planner import dijkstra, route_waypoints       # noqa: E402

EPS = 1e-6

# Collected pass/fail, for the script runner.
_results = []


def check(cond, msg):
    _results.append((bool(cond), msg))
    print(f"{'PASS' if cond else 'FAIL'}  {msg}")
    return bool(cond)


# A small deterministic PRNG (mulberry32) so the random / disconnected graph fuzz
# is reproducible: the same run every time, so a failure can be replayed instead
# of vanishing on the next invocation. Ported verbatim from the original test so
# the exact same graph sequence is exercised.
_seed = 0x9E3779B9


def _u32(x):
    return x & 0xFFFFFFFF


def _imul(a, b):
    return _u32((a & 0xFFFFFFFF) * (b & 0xFFFFFFFF))


def rand():
    global _seed
    _seed = _u32(_seed + 0x6D2B79F5)
    t = _imul(_seed ^ (_seed >> 15), 1 | _seed)
    t = _u32((t + _imul(t ^ (t >> 7), 61 | t)) ^ t)
    return ((t ^ (t >> 14)) & 0xFFFFFFFF) / 4294967296.0


def _reset_rand():
    global _seed
    _seed = 0x9E3779B9


# ----- independent oracle: Floyd-Warshall all-pairs shortest paths -----
def floyd(nodes, edges):
    n = len(nodes)
    D = [[math.inf] * n for _ in range(n)]
    for i in range(n):
        D[i][i] = 0.0
    for a, b in edges:
        w = dist(nodes[a], nodes[b])
        D[a][b] = min(D[a][b], w)
        D[b][a] = min(D[b][a], w)
    for k in range(n):
        for i in range(n):
            dik = D[i][k]
            if dik == math.inf:
                continue
            Dk = D[k]
            Di = D[i]
            for j in range(n):
                nd = dik + Dk[j]
                if nd < Di[j]:
                    Di[j] = nd
    return D


def path_is_valid(adj, src, dst, res):
    """A reported path must be a real walk: each step is an edge, ends right,
    and the edge lengths add up to the reported distance."""
    if len(res.path) == 0:
        return not math.isfinite(res.dist)
    if res.path[0] != src or res.path[-1] != dst:
        return False
    total = 0.0
    for i in range(len(res.path) - 1):
        u, v = res.path[i], res.path[i + 1]
        edge = next((e for e in adj[u] if e.to == v), None)
        if edge is None:
            return False                       # not an adjacent pair: invalid
        total += edge.w
    return abs(total - res.dist) < 1e-6


class _P:
    """A tiny point with .x/.y for the oracle distance helper."""

    __slots__ = ("id", "x", "y")

    def __init__(self, i, x, y):
        self.id, self.x, self.y = i, x, y


# ======================= 1. map integrity =======================
def test_map_integrity():
    adj = town.adjacency()

    place_ids = [town.node_of(p).id if town.node_of(p) else None for p in town.PLACES]
    check(all(i is not None for i in place_ids), "every named place resolves to a node")
    check(len(set(place_ids)) == len(town.PLACES), "place nodes are distinct")

    symmetric = True
    for u in range(len(adj)):
        for e in adj[u]:
            back = next((b for b in adj[e.to] if b.to == u), None)
            if back is None or abs(back.w - e.w) > EPS:
                symmetric = False
    check(symmetric, "road graph is undirected (symmetric adjacency, equal weights)")

    depot = town.node_of("depot").id
    reach = all(math.isfinite(dijkstra(adj, depot, n.id).dist) for n in town.NODES)
    check(reach, "every node is reachable from the depot")


# ============= 2. Dijkstra vs Floyd-Warshall on the real map =============
def test_dijkstra_vs_floyd_map():
    adj = town.adjacency()
    D = floyd(town.NODES, town.EDGES)
    dist_ok = path_ok = True
    for s in range(len(town.NODES)):
        for t in range(len(town.NODES)):
            res = dijkstra(adj, s, t)
            if abs(res.dist - D[s][t]) > 1e-6:
                dist_ok = False
            if not path_is_valid(adj, s, t, res):
                path_ok = False
    check(dist_ok, "Dijkstra matches Floyd-Warshall for all node pairs on the map")
    check(path_ok, "every reconstructed path is a valid walk with the reported length")


# ===================== 3. unreachable goal =====================
def test_unreachable_goal():
    nodes = [_P(0, 0, 0), _P(1, 10, 0)]
    adj = build_adj(nodes, [])
    res = dijkstra(adj, 0, 1)
    check(not math.isfinite(res.dist) and len(res.path) == 0,
          "unreachable goal returns Infinity and empty path")


# ============ 4. random-graph fuzz: Dijkstra vs Floyd-Warshall ============
def test_random_graph_fuzz():
    _reset_rand()

    def rnd(a, b):
        return a + rand() * (b - a)

    mismatches = invalid_paths = trials = 0
    for _g in range(250):
        n = 4 + int(rand() * 9)                          # 4..12 nodes
        nodes = [_P(i, rnd(0, 500), rnd(0, 500)) for i in range(n)]

        edges = []
        for i in range(1, n):                            # spanning tree -> connected
            edges.append((int(rand() * i), i))
        seen = set()
        for a, b in edges:
            seen.add((a, b) if a < b else (b, a))
        extra = int(rand() * n)
        for _k in range(extra):
            a, b = int(rand() * n), int(rand() * n)
            if a == b:
                continue
            key = (a, b) if a < b else (b, a)
            if key in seen:
                continue
            seen.add(key)
            edges.append((a, b))

        adj = build_adj(nodes, edges)
        D = floyd(nodes, edges)
        for s in range(n):
            for t in range(n):
                res = dijkstra(adj, s, t)
                trials += 1
                if abs(res.dist - D[s][t]) > 1e-6:
                    mismatches += 1
                if not path_is_valid(adj, s, t, res):
                    invalid_paths += 1

    check(mismatches == 0, f"random graphs: Dijkstra == Floyd-Warshall on all {trials} pairs")
    check(invalid_paths == 0, "random graphs: every reconstructed path is a valid walk")


# ====== 4b. disconnected graphs: the unreachable branch, fuzzed ======
def test_disconnected_graph_fuzz():
    def rnd(a, b):
        return a + rand() * (b - a)

    bad_inf = bad_valid = cross_pairs = 0
    for _g in range(120):
        nA = 2 + int(rand() * 4)                         # component A: ids 0..nA-1
        nB = 2 + int(rand() * 4)                         # component B: ids nA..n-1
        n = nA + nB
        nodes = [_P(i, rnd(0, 400), rnd(0, 400)) for i in range(n)]

        edges = []
        for i in range(1, nA):
            edges.append((int(rand() * i), i))
        for i in range(nA + 1, n):
            edges.append((nA + int(rand() * (i - nA)), i))

        adj = build_adj(nodes, edges)
        comp = lambda i: 0 if i < nA else 1
        for s in range(n):
            for t in range(n):
                res = dijkstra(adj, s, t)
                if comp(s) != comp(t):
                    cross_pairs += 1
                    if math.isfinite(res.dist) or len(res.path) != 0:
                        bad_inf += 1
                elif not path_is_valid(adj, s, t, res):
                    bad_valid += 1

    check(bad_inf == 0,
          f"disconnected graphs: all {cross_pairs} cross-component pairs return Infinity + empty path")
    check(bad_valid == 0, "disconnected graphs: in-component paths stay valid walks")


# ============= 5. steering + kinematics turn the right way =============
def test_steering_and_kinematics():
    def drive(vL, vR, steps):
        car = Car(0.0, 0.0, 0.0)   # heading +x; +y is the car's right on this y-down map
        for _ in range(steps):
            step_kinematics(car, vL, vR)
        return car

    r = drive(1, 0, 12)            # faster left wheel
    check(r.y > 0 and r.theta > 0, "faster left wheel -> car curves toward its right (+y, world)")
    l = drive(0, 1, 12)            # faster right wheel
    check(l.y < 0 and l.theta < 0, "faster right wheel -> car curves toward its left (-y, world)")

    # Steering: a target above the car (y<0, the car's left when heading +x) must
    # ease the LEFT wheel so the car turns to face it.
    car_l = Car(0.0, 0.0, 0.0)
    vL, vR, err = steer_toward(car_l, (30, -30), 1.0)
    check(err < 0 and vL < vR, "target to the left -> left wheel eased, car turns left toward it")
    vL, vR, err = steer_toward(car_l, (30, 30), 1.0)
    check(err > 0 and vR < vL, "target to the right -> right wheel eased, car turns right toward it")

    # Driven a few steps, the heading error toward an off-axis target must shrink.
    car = Car(0.0, 0.0, 0.0)
    _, _, err0 = steer_toward(car, (60, 60), 1.0)
    for _ in range(20):
        a, b, _ = steer_toward(car, (60, 60), 1.0)
        step_kinematics(car, a, b)
    _, _, err1 = steer_toward(car, (60, 60), 1.0)
    check(abs(err1) < abs(err0), "steering reduces the heading error over time")

    # Inner-wheel clamp: even at a target nearly behind the car (heading error ~ pi),
    # and at the fastest speed, neither wheel may run backwards.
    bL, bR, _ = steer_toward(Car(0.0, 0.0, 0.0), (-50, 1), 2.5)
    check(bL >= 0 and bR >= 0, "sharp turn keeps both wheels >= 0 (inner wheel never reverses)")


# ===== 6. the robot delivers to every place, across the speed range =====
def test_delivers_every_place_all_speeds():
    adj = town.adjacency()
    depot = town.node_of("depot").id
    # Half the painted road is roadWidth/2 = 7.5px. The robot's center must stay
    # inside that band at EVERY speed, not just a few samples -- the worst leg is
    # not at an endpoint of the range (it peaks around 2.4x), so we sweep the
    # whole 0.3 .. 2.5 range in 0.1 steps.
    CROSS_LIMIT = 7
    speeds = [round(v * 10) / 10 for v in
              (0.3 + 0.1 * i for i in range(int((2.5 - 0.3) / 0.1 + 0.5) + 1))]

    legs = []
    for p in town.PLACES:
        if p == "depot":
            continue
        legs.append((depot, town.node_of(p).id))   # out
        legs.append((town.node_of(p).id, depot))    # back

    for speed in speeds:
        all_arrived = True
        worst_cross = worst_end = 0.0
        for s, t in legs:
            wp = route_waypoints(town.NODES, dijkstra(adj, s, t).path)
            res = drive_route(wp, speed=speed)
            if not res.arrived:
                all_arrived = False
            worst_cross = max(worst_cross, res.max_cross_track)
            worst_end = max(worst_end, res.end_dist)
        check(all_arrived and worst_end <= ARRIVE_RADIUS,
              f"{speed}x: reaches every goal (worst stop {worst_end:.1f}px <= {ARRIVE_RADIUS:.0f}px)")
        check(worst_cross <= CROSS_LIMIT,
              f"{speed}x: stays on the road (worst cross-track {worst_cross:.1f}px <= {CROSS_LIMIT}px)")


_ALL_TESTS = [
    test_map_integrity,
    test_dijkstra_vs_floyd_map,
    test_unreachable_goal,
    test_random_graph_fuzz,
    test_disconnected_graph_fuzz,
    test_steering_and_kinematics,
    test_delivers_every_place_all_speeds,
]


def main():
    for fn in _ALL_TESTS:
        fn()
    failed = sum(1 for ok, _ in _results if not ok)
    total = len(_results)
    if failed:
        print(f"\n{failed} of {total} check(s) failed.")
        return 1
    print(f"\nAll {total} checks passed: planner matches the oracle, "
          "paths are valid, the robot delivers.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
