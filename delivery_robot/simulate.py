"""The tending loop: plan -> drive -> drop -> return.

A small state machine ties the planner and the controller together. The robot
starts idle at the supply station. When a plant is chosen it picks up a unit of
water or fertilizer, plans station -> plant, and drives there. On arrival it
drops the unit off -- one watering or one fertilizing -- plans plant -> station,
and drives home, where it goes idle again, ready for the next job.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from . import town
from .controller import Car, follow_step, start_pose
from .planner import dijkstra, route_waypoints


# Tending phases.
IDLE = "idle"
TO_DEST = "toDest"
TO_DEPOT = "toDepot"


@dataclass
class Sim:
    """Live state of one tending run, advanced one frame at a time."""

    adj: list
    speed: float = 1.0
    src: int = 0
    dst: int | None = None
    route: list = field(default_factory=list)        # node ids
    wp: list = field(default_factory=list)           # waypoints
    car: Car | None = None
    target: int = 1
    phase: str = IDLE
    carrying: bool = False
    route_len: float = 0.0

    @classmethod
    def new(cls, speed: float = 1.0):
        s = cls(adj=town.adjacency(), speed=speed, src=town.node_of("depot").id)
        s.reset_to_depot()
        return s

    def _plan_leg(self, src_id: int, dst_id: int) -> None:
        r = dijkstra(self.adj, src_id, dst_id)
        self.route = r.path
        self.wp = route_waypoints(town.NODES, r.path)
        self.route_len = r.dist
        self.car = start_pose(self.wp)
        self.target = 1

    def send_to(self, dst_id: int) -> None:
        """Dispatch the robot to tend the plant at ``dst_id``."""
        depot = town.node_of("depot").id
        if dst_id == depot:
            self.reset_to_depot()
            return
        self.dst = dst_id
        self.carrying = True
        self.phase = TO_DEST
        self._plan_leg(depot, dst_id)

    def reset_to_depot(self) -> None:
        """Park empty at the supply station, ready for the next job."""
        node = town.NODES[self.src]
        self.dst = None
        self.carrying = False
        self.phase = IDLE
        self.route = []
        self.wp = []
        self.route_len = 0.0
        self.car = Car(x=node.x, y=node.y, theta=-math.pi / 2)

    def step(self) -> None:
        """Advance the robot one frame along its current leg; handle transitions."""
        if self.phase == IDLE or len(self.wp) < 2:
            return
        self.target, finished = follow_step(self.car, self.wp, self.target, self.speed)
        if not finished:
            return
        # Reached the end of this leg.
        if self.phase == TO_DEST:
            self.carrying = False                 # dropped off the water/fertilizer
            self.phase = TO_DEPOT
            self._plan_leg(self.dst, town.node_of("depot").id)
        elif self.phase == TO_DEPOT:
            self.reset_to_depot()

    def status_line(self) -> str:
        """A short description of what the robot is doing right now."""
        if self.phase == IDLE:
            return "At the supply station. Pick a plant."
        place = town.NODES[self.dst].place
        label = town.label_of(place)
        cargo = town.cargo_of(place)              # "water" or "fertilizer"
        if self.phase == TO_DEST:
            return f"Carrying {cargo} to the {label}"
        action = "Watered" if cargo == "water" else "Fertilized"
        return f"{action} the {label} - returning to the supply station."
