"""Rendering with Pillow: static garden PNGs and the animated tending GIF.

Everything is drawn with ``PIL.Image`` + ``PIL.ImageDraw`` and plain numpy math.
Pillow can save an animated GIF directly, so no external animation library is
needed.
"""

from __future__ import annotations

import math
import os

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from . import town
from .controller import Car
from .simulate import IDLE, TO_DEPOT, TO_DEST, Sim

# --- garden image geometry ---
WIDTH = 820
HEIGHT = 600
MARGIN = 11                      # border around the map, matches the saved PNGs
ROAD_WIDTH = 15
CAR_LENGTH = 26
CAR_WIDTH = 16
STATUS_H = 34                    # status strip drawn under the map

COLORS = {
    "bg": (243, 239, 230),
    "road": (207, 214, 221),
    "road_edge": (183, 192, 201),
    "route": (31, 111, 235),
    "node": (154, 166, 178),
    "depot": (15, 118, 110),
    "place": (180, 83, 9),
    "place_active": (234, 88, 8),
    "car": (31, 111, 235),
    "car_edge": (11, 61, 145),
    "parcel": (245, 158, 11),
    "parcel_edge": (146, 64, 14),
    "nose": (253, 224, 71),
    "label": (27, 35, 48),
    "white": (255, 255, 255),
    "status_bg": (255, 255, 255),
    "status_text": (40, 48, 60),
}


def _font(size: int):
    """A sans-serif font if one is on the system, else Pillow's default bitmap."""
    for name in (
        "DejaVuSans.ttf",
        "/System/Library/Fonts/Helvetica.ttc",
        "/System/Library/Fonts/Supplemental/Arial.ttf",
        "Arial.ttf",
    ):
        try:
            return ImageFont.truetype(name, size)
        except (OSError, IOError):
            continue
    return ImageFont.load_default()


def _xy(p):
    """Offset a map point by the border margin."""
    return (p[0] + MARGIN, p[1] + MARGIN)


def _node_xy(n):
    return (n.x + MARGIN, n.y + MARGIN)


def _draw_paths(draw: ImageDraw.ImageDraw) -> None:
    # casing first, then the lighter surface on top
    for a, b in town.EDGES:
        draw.line([_node_xy(town.NODES[a]), _node_xy(town.NODES[b])],
                  fill=COLORS["road_edge"], width=ROAD_WIDTH + 4, joint="curve")
    for a, b in town.EDGES:
        draw.line([_node_xy(town.NODES[a]), _node_xy(town.NODES[b])],
                  fill=COLORS["road"], width=ROAD_WIDTH, joint="curve")
    # round caps at every junction so corners look filled
    r = (ROAD_WIDTH + 4) / 2
    for n in town.NODES:
        cx, cy = _node_xy(n)
        draw.ellipse([cx - r, cy - r, cx + r, cy + r], fill=COLORS["road_edge"])
    r = ROAD_WIDTH / 2
    for n in town.NODES:
        cx, cy = _node_xy(n)
        draw.ellipse([cx - r, cy - r, cx + r, cy + r], fill=COLORS["road"])


def _draw_route(draw: ImageDraw.ImageDraw, wp) -> None:
    """Draw the planned route as a dashed blue polyline."""
    if len(wp) < 2:
        return
    dash, gap = 2, 7
    for i in range(len(wp) - 1):
        ax, ay = _xy(wp[i])
        bx, by = _xy(wp[i + 1])
        seg = math.hypot(bx - ax, by - ay)
        if seg < 1e-9:
            continue
        ux, uy = (bx - ax) / seg, (by - ay) / seg
        t = 0.0
        while t < seg:
            t2 = min(t + dash, seg)
            draw.line([(ax + ux * t, ay + uy * t), (ax + ux * t2, ay + uy * t2)],
                      fill=COLORS["route"], width=5)
            t += dash + gap


def _draw_nodes(draw: ImageDraw.ImageDraw) -> None:
    for n in town.NODES:
        if n.place:
            continue
        cx, cy = _node_xy(n)
        draw.ellipse([cx - 4, cy - 4, cx + 4, cy + 4], fill=COLORS["node"])


def _draw_places(draw: ImageDraw.ImageDraw, active_dst, font) -> None:
    for n in town.NODES:
        if not n.place:
            continue
        cx, cy = _node_xy(n)
        is_depot = n.place == "depot"
        is_active = active_dst == n.id
        r = 11 if is_depot else 9
        fill = COLORS["depot"] if is_depot else (
            COLORS["place_active"] if is_active else COLORS["place"])
        draw.ellipse([cx - r, cy - r, cx + r, cy + r],
                     fill=fill, outline=COLORS["white"], width=2)
        # English label only; nothing non-ASCII is rendered into the image.
        label = town.label_of(n.place)
        ly = cy - 18 if n.y > HEIGHT - 40 else cy + 12
        tb = draw.textbbox((0, 0), label, font=font)
        tw = tb[2] - tb[0]
        draw.text((cx - tw / 2, ly), label, fill=COLORS["label"], font=font)


def _to_world(local_pts, cx, cy, theta):
    """Rotate body-frame points by ``theta`` and translate to ``(cx, cy)``.

    ``local_pts`` is an (N, 2) array of offsets in the car's own frame. The 2x2
    rotation matches the kinematics: +x is the car's nose, +y its right.
    """
    pts = np.asarray(local_pts, dtype=float)
    c, s = math.cos(theta), math.sin(theta)
    rot = np.array([[c, -s], [s, c]])
    world = pts @ rot.T + np.array([cx, cy])
    return [tuple(p) for p in world]


def _draw_car(draw: ImageDraw.ImageDraw, car: Car, carrying: bool) -> None:
    cx, cy = car.x + MARGIN, car.y + MARGIN
    L, W = CAR_LENGTH, CAR_WIDTH

    body = _to_world([(-L / 2, -W / 2), (L / 2, -W / 2),
                      (L / 2, W / 2), (-L / 2, W / 2)], cx, cy, car.theta)
    draw.polygon(body, fill=COLORS["car"], outline=COLORS["car_edge"])
    # nose triangle so the heading is obvious
    nose = _to_world([(L / 2, 0), (L / 2 - 7, -5), (L / 2 - 7, 5)], cx, cy, car.theta)
    draw.polygon(nose, fill=COLORS["nose"])
    # the water/fertilizer unit (a small block) on the robot while carrying
    if carrying:
        block = _to_world([(-6, -6), (6, -6), (6, 6), (-6, 6)], cx, cy, car.theta)
        draw.polygon(block, fill=COLORS["parcel"], outline=COLORS["parcel_edge"])


def _draw_status(draw: ImageDraw.ImageDraw, text: str, font) -> None:
    y0 = MARGIN + HEIGHT + MARGIN
    draw.rectangle([0, y0, WIDTH + 2 * MARGIN, y0 + STATUS_H], fill=COLORS["status_bg"])
    draw.text((MARGIN, y0 + 8), text, fill=COLORS["status_text"], font=font)


def _base_frame(font, *, with_status: bool):
    h = HEIGHT + 2 * MARGIN + (STATUS_H if with_status else 0)
    img = Image.new("RGB", (WIDTH + 2 * MARGIN, h), COLORS["bg"])
    draw = ImageDraw.Draw(img)
    _draw_paths(draw)
    return img, draw


def render_overview(path: str) -> None:
    """The garden map with no route or robot: media/map-overview.png."""
    font = _font(13)
    img, draw = _base_frame(font, with_status=False)
    _draw_nodes(draw)
    _draw_places(draw, active_dst=None, font=font)
    img.save(path)


def render_route_still(path: str, destination: str) -> None:
    """A single still of a planned tending run: media/delivery-route.png."""
    from .planner import dijkstra, route_waypoints

    font = _font(13)
    sim = Sim.new()
    sim.send_to(town.node_of(destination).id)
    # drive partway so the robot sits on the route, carrying its unit
    for _ in range(220):
        sim.step()
        if sim.phase != TO_DEST:
            break
    img, draw = _base_frame(font, with_status=False)
    _draw_route(draw, sim.wp)
    _draw_nodes(draw)
    _draw_places(draw, active_dst=sim.dst, font=font)
    if sim.car is not None:
        _draw_car(draw, sim.car, sim.carrying)
    img.save(path)


def render_gif(path: str, destinations=None, frame_skip: int = 6,
               duration_ms: int = 60, max_frames: int = 400,
               sim_speed: float = 1.6) -> int:
    """Render an animated GIF of one or more full tending runs.

    The robot picks up at the supply station, drives to each plant in turn,
    drops off the water/fertilizer unit and returns, with the dashed planned
    route, the carried unit and a status line drawn each frame. ``frame_skip``
    keeps every n-th simulation step as a frame, ``sim_speed`` runs the
    controller faster so a full out-and-back fits in a compact clip. Returns the
    number of frames written.
    """
    if destinations is None:
        # one fertilizing run (Tomatoes) then one watering run (Roses), so the
        # GIF shows both kinds of trip.
        destinations = ["tomatoes", "roses"]

    font = _font(13)
    status_font = _font(15)
    frames = []

    for dest in destinations:
        sim = Sim.new(speed=sim_speed)
        sim.send_to(town.node_of(dest).id)
        tick = 0
        guard = 0
        # run until the robot is back home and idle
        while guard < max_frames * frame_skip:
            if tick % frame_skip == 0:
                img, draw = _base_frame(font, with_status=True)
                _draw_route(draw, sim.wp)
                _draw_nodes(draw)
                _draw_places(draw, active_dst=sim.dst, font=font)
                if sim.car is not None:
                    _draw_car(draw, sim.car, sim.carrying)
                _draw_status(draw, sim.status_line(), status_font)
                frames.append(img)
                if len(frames) >= max_frames:
                    break
            sim.step()
            tick += 1
            guard += 1
            if sim.phase == IDLE and tick > 1:
                # capture one last idle frame, then move on
                img, draw = _base_frame(font, with_status=True)
                _draw_nodes(draw)
                _draw_places(draw, active_dst=None, font=font)
                if sim.car is not None:
                    _draw_car(draw, sim.car, sim.carrying)
                _draw_status(draw, sim.status_line(), status_font)
                frames.append(img)
                break
        if len(frames) >= max_frames:
            break

    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    frames[0].save(path, save_all=True, append_images=frames[1:],
                   duration=duration_ms, loop=0, optimize=True)
    return len(frames)


def render_all(media_dir: str = "media") -> None:
    """(Re)generate the static PNGs and the GIF."""
    render_overview(os.path.join(media_dir, "map-overview.png"))
    render_route_still(os.path.join(media_dir, "delivery-route.png"), "tomatoes")
    n = render_gif(os.path.join(media_dir, "delivery.gif"))
    print(f"wrote {media_dir}/delivery.gif ({n} frames)")


if __name__ == "__main__":
    import sys

    target = sys.argv[1] if len(sys.argv) > 1 else "media"
    render_all(target)
