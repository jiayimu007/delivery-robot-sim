"""Local route-following: proportional steering + differential-drive motion.

The robot follows a planned route one junction at a time. It steers toward the
next junction with a proportional controller (turn harder the more its heading
is off), eases off the throttle before a sharp corner, and a differential-drive
model turns the two wheel speeds into motion.

The garden's y-axis points down (image convention). With ``x += v*cos(theta)``
and ``y += v*sin(theta)``, a positive yaw rotates the heading from +x toward +y,
so a faster left wheel curves the robot toward its slower right wheel, exactly
like a real two-wheel robot. The steering math uses the same ``atan2`` convention
as the motion, so steering ``theta`` toward the desired heading points the robot
straight at its target with no extra axis bookkeeping.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


# --- tuning constants (the same values the simulation was tuned with) ---
BASE_SPEED = 1.6        # forward speed at the 1.0 speed setting (px / frame)
STEER_GAIN = 2.0        # wheel-speed bias per radian of heading error
WHEEL_BASE = 16.0       # distance between the wheels (px)
ARRIVE_RADIUS = 9.0     # px: close enough to a junction to move on to the next
ERR_SLOW = 0.5          # ease off the throttle by up to this much on heading error
TURN_SLOW = 0.8         # ease off by up to this much approaching a sharp turn
SLOW_DIST = 140.0       # px: how far ahead of a junction the turn slowdown starts


@dataclass
class Car:
    """The robot's pose: position and heading (radians)."""

    x: float
    y: float
    theta: float


def wrap_pi(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def _hypot_xy(ax, ay, bx, by):
    return math.hypot(ax - bx, ay - by)


def steer_toward(car: Car, target, speed: float, nxt=None):
    """Proportional steering toward ``target`` (an ``(x, y)`` pair).

    ``speed`` is the speed multiplier; ``nxt`` is the junction after ``target``
    (if any), used to see a turn coming and slow before the corner. Returns
    ``(vL, vR, err)``: the two wheel speeds and the heading error.

    The desired heading is the angle from the car to the target; the heading
    error is how far the car is turned away from it. The wheels are biased by
    that error, so the car swings until it points at the target. Two things ease
    off the throttle so corners stay inside the lane: a large current heading
    error, and a sharp turn coming up at the junction being approached. Slowing
    before the corner (not just reacting after it) is what keeps the car in its
    lane at high speed.
    """
    tx, ty = target
    err = wrap_pi(math.atan2(ty - car.y, tx - car.x) - car.theta)

    slow_err = 1.0 - ERR_SLOW * min(1.0, abs(err) / (math.pi / 2))
    slow_turn = 1.0
    if nxt is not None:
        nx, ny = nxt
        into = math.atan2(ty - car.y, tx - car.x)
        out_of = math.atan2(ny - ty, nx - tx)
        turn = abs(wrap_pi(out_of - into))                 # angle of the upcoming turn
        d_to_target = _hypot_xy(car.x, car.y, tx, ty)
        prox = max(0.0, min(1.0, 1.0 - d_to_target / SLOW_DIST))
        slow_turn = 1.0 - TURN_SLOW * (turn / math.pi) * prox

    base = BASE_SPEED * speed * min(slow_err, slow_turn)

    # Wheel-speed bias, clamped to +/- base so the inner wheel never reverses.
    # vL - vR has the same sign as err, and omega = (vL - vR) / b, so theta moves
    # toward the desired heading: the car turns to face the target.
    diff = STEER_GAIN * err
    diff = max(-base, min(base, diff))

    return base + diff, base - diff, err


def step_kinematics(car: Car, vL: float, vR: float) -> None:
    """Differential-drive kinematics: advance the pose by one step in place.

    Forward speed is the average of the two wheels; yaw rate is their difference
    divided by the wheel base. A faster left wheel (vL > vR) gives omega > 0,
    which rotates the heading from +x toward +y, so the car curves toward its
    slower right wheel.
    """
    v = (vL + vR) / 2.0
    omega = (vL - vR) / WHEEL_BASE
    car.theta += omega
    car.x += v * math.cos(car.theta)
    car.y += v * math.sin(car.theta)


def point_to_path(px, py, poly) -> float:
    """Shortest distance from a point to an open polyline (the planned route).

    Used to measure how well the robot tracks its route (cross-track error).
    ``poly`` is a list of ``(x, y)`` waypoints.
    """
    best = math.inf
    for i in range(len(poly) - 1):
        ax, ay = poly[i]
        bx, by = poly[i + 1]
        dx, dy = bx - ax, by - ay
        len2 = dx * dx + dy * dy or 1e-9
        t = ((px - ax) * dx + (py - ay) * dy) / len2
        t = max(0.0, min(1.0, t))
        ex = ax + t * dx - px
        ey = ay + t * dy - py
        best = min(best, ex * ex + ey * ey)
    return math.sqrt(best)


def start_pose(wp) -> Car:
    """Pose at the start of a route: sit on the first waypoint, face the second."""
    p0 = wp[0]
    p1 = wp[1] if len(wp) > 1 else wp[0]
    return Car(x=p0[0], y=p0[1], theta=math.atan2(p1[1] - p0[1], p1[0] - p0[0]))


def follow_step(car: Car, wp, target: int, speed: float):
    """One step of route-following.

    Advance past any waypoint already reached, then steer toward the next one and
    move. Returns ``(target, finished)`` where ``target`` is the (possibly
    advanced) index of the waypoint the car is heading for.
    """
    while target < len(wp) and _hypot_xy(car.x, car.y, wp[target][0], wp[target][1]) < ARRIVE_RADIUS:
        target += 1
    if target >= len(wp):
        return target, True
    nxt = wp[target + 1] if target + 1 < len(wp) else None
    vL, vR, _ = steer_toward(car, wp[target], speed, nxt)
    step_kinematics(car, vL, vR)
    return target, False


@dataclass
class DriveResult:
    arrived: bool
    steps: int
    max_cross_track: float
    end_dist: float


def drive_route(wp, speed: float = 1.0, max_steps: int = 8000) -> DriveResult:
    """Drive along a waypoint route until it reaches the end (or runs out of steps).

    Pure logic, no drawing. Returns a ``DriveResult`` with whether it arrived,
    how many steps it took, the worst cross-track error, and the final distance
    to the goal.
    """
    if len(wp) < 2:
        return DriveResult(arrived=True, steps=0, max_cross_track=0.0, end_dist=0.0)

    car = start_pose(wp)
    target = 1
    max_cross = 0.0
    steps = 0

    while steps < max_steps:
        target, finished = follow_step(car, wp, target, speed)
        if finished:
            break
        cross = point_to_path(car.x, car.y, wp)
        if cross > max_cross:
            max_cross = cross
        steps += 1

    end = wp[-1]
    end_dist = _hypot_xy(car.x, car.y, end[0], end[1])
    return DriveResult(
        arrived=end_dist < ARRIVE_RADIUS * 1.6,
        steps=steps,
        max_cross_track=max_cross,
        end_dist=end_dist,
    )
