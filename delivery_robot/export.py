"""Export the engine's results as JSON for the browser player.

The browser demo does **not** re-run the planner or the controller. Instead
this module runs the real Python engine -- Dijkstra in :mod:`planner` and the
proportional steering / differential-drive controller in :mod:`controller`,
driven by the tending state machine in :mod:`simulate` -- and writes its
output to ``web/data/`` as plain JSON:

  * ``town.json``  -- the garden-path graph: nodes (id, English label, the
    water/fertilizer the bed receives, x, y) and edges, plus a little metadata
    (canvas size, supply-station id, the list of plants).
  * ``routes/<place>.json`` -- for each plant, the planned path the engine
    computed (the node-id sequence for both legs) and the robot's per-frame
    trajectory ``[{x, y, theta, carrying}, ...]`` recorded one entry per
    simulation step for the full out-and-back tending run.

Every number here comes from the engine. The path is whatever Dijkstra
returned; the trajectory is whatever the controller drove. Nothing is
hand-written. Re-run this whenever the engine changes:

    python -m delivery_robot.export web/data

so the browser animation stays in lock-step with the tested Python code.

Only English plant/station labels are written, matching what the renderer paints
into the GIF; the file carries no other labels.
"""

from __future__ import annotations

import json
import os

from . import town
from .simulate import IDLE, TO_DEST, Sim

# How fast the engine is driven when recording a trajectory. This matches the
# GIF (``render.render_gif`` uses ``sim_speed=1.6``) so the exported frames look
# the same as the preview. The player's speed slider only changes playback rate;
# it never re-drives the engine.
EXPORT_SPEED = 1.6

# A hard ceiling on recorded steps per delivery, so a hypothetical stall cannot
# produce an unbounded file. A full out-and-back is a few hundred steps.
MAX_STEPS = 6000


def _round(v: float, ndigits: int = 2) -> float:
    """Round and normalise ``-0.0`` to ``0.0`` so the JSON is tidy."""
    r = round(float(v), ndigits)
    return 0.0 if r == 0.0 else r


def town_dict() -> dict:
    """The garden-path graph as a JSON-ready dict: nodes, edges and metadata.

    Each node carries its short key (``place``), its English display ``label``
    and, for a plant bed, the ``cargo`` it receives (``"water"`` or
    ``"fertilizer"``); plain junctions have ``place: null``. Positions are the
    same pixel coordinates the engine and the renderer use, so the player can
    draw the garden without any geometry of its own.
    """
    nodes = [
        {
            "id": n.id,
            "x": _round(n.x),
            "y": _round(n.y),
            "place": n.place,
            "label": town.label_of(n.place) if n.place else None,
            "cargo": town.cargo_of(n.place) if n.place else "",
        }
        for n in town.NODES
    ]
    edges = [[a, b] for (a, b) in town.EDGES]
    depot = town.node_of("depot")
    destinations = [p for p in town.PLACES if p != "depot"]
    return {
        # canvas size matches render.WIDTH/HEIGHT + the margin used for the PNGs,
        # so the exported coordinates land in the same frame the GIF uses.
        "width": 820,
        "height": 600,
        "margin": 11,
        # the supply station the robot starts and ends each run at
        "depot": depot.id,
        "places": town.PLACES,
        "destinations": destinations,
        "nodes": nodes,
        "edges": edges,
    }


def record_delivery(place: str) -> dict:
    """Run the engine for one full tending run to ``place`` and capture it.

    Drives the real :class:`~delivery_robot.simulate.Sim` (Dijkstra plan + the
    controller) from the supply station to ``place`` and back, recording one
    frame per simulation step. Returns the two-leg planned node paths and the
    per-frame trajectory. The numbers are produced entirely by the engine.
    """
    dst = town.node_of(place)
    if dst is None:
        raise ValueError(f"unknown plant: {place!r}")

    sim = Sim.new(speed=EXPORT_SPEED)
    sim.send_to(dst.id)

    # The planned path for the outbound leg, straight from Dijkstra.
    path_to_dest = list(sim.route)
    path_to_depot: list = []

    frames = []

    def snapshot() -> None:
        car = sim.car
        frames.append({
            "x": _round(car.x),
            "y": _round(car.y),
            "theta": _round(car.theta, 4),
            "carrying": bool(sim.carrying),
        })

    snapshot()  # the starting pose at the supply station, unit picked up
    prev_phase = sim.phase
    steps = 0
    while steps < MAX_STEPS:
        sim.step()
        # The state machine re-plans when it switches from the outbound leg to
        # the return leg; grab that return path the moment it appears.
        if prev_phase == TO_DEST and sim.phase != TO_DEST and not path_to_depot:
            path_to_depot = list(sim.route)
        prev_phase = sim.phase
        snapshot()
        steps += 1
        if sim.phase == IDLE:
            break

    return {
        "place": place,
        "label": town.label_of(place),       # English display name of the plant
        "cargo": town.cargo_of(place),        # "water" or "fertilizer"
        "destId": dst.id,
        "pathToDest": path_to_dest,
        "pathToDepot": path_to_depot,
        "frames": frames,
    }


def export(out_dir: str = "web/data") -> dict:
    """Write ``town.json`` and one route file per destination under ``out_dir``.

    Returns a small manifest (the same data the player's index reads) describing
    what was written, so the call site can print or check it.
    """
    routes_dir = os.path.join(out_dir, "routes")
    os.makedirs(routes_dir, exist_ok=True)

    # The graph file keeps the historical name town.json so existing URLs and the
    # player's fetch path do not change; its contents are now the garden graph.
    town_data = town_dict()
    with open(os.path.join(out_dir, "town.json"), "w", encoding="utf-8") as f:
        json.dump(town_data, f, ensure_ascii=True, indent=1)
        f.write("\n")

    manifest_routes = []
    for place in town_data["destinations"]:
        rec = record_delivery(place)
        fname = f"{place}.json"
        with open(os.path.join(routes_dir, fname), "w", encoding="utf-8") as f:
            json.dump(rec, f, ensure_ascii=True, indent=1)
            f.write("\n")
        manifest_routes.append({
            "place": place,
            "file": f"routes/{fname}",
            "frames": len(rec["frames"]),
        })

    manifest = {"town": "town.json", "routes": manifest_routes}
    with open(os.path.join(out_dir, "index.json"), "w", encoding="utf-8") as f:
        json.dump(manifest, f, ensure_ascii=True, indent=1)
        f.write("\n")
    return manifest


def main(argv=None) -> int:
    import sys

    argv = list(sys.argv[1:] if argv is None else argv)
    out_dir = argv[0] if argv else "web/data"
    manifest = export(out_dir)
    total = sum(r["frames"] for r in manifest["routes"])
    print(f"wrote {out_dir}/town.json and {len(manifest['routes'])} route files "
          f"({total} frames total)")
    for r in manifest["routes"]:
        print(f"  {r['place']:<8} {r['frames']:>4} frames  ({r['file']})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
