"""Garden-tending robot navigation simulation.

A small robot drives around an abstract garden and carries a unit of water or
fertilizer from the supply station to a chosen plant. It works in two layers:

  1. Planning (global): the garden paths are a weighted graph and Dijkstra's
     algorithm finds the shortest route from the supply station to the plant.
  2. Driving (local): a proportional steering controller with pre-corner
     deceleration follows that route junction by junction, and a
     differential-drive model turns the two wheel speeds into motion.

This is a software model written to study the idea. It is not firmware for a
real robot.
"""

from __future__ import annotations

from . import controller, graph, planner, render, simulate, town

__all__ = ["graph", "town", "planner", "controller", "simulate", "render"]
