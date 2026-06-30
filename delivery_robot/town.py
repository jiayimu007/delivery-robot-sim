"""The abstract garden the robot tends.

Nodes are path junctions (and the named plant beds); edges are two-way garden
paths. Positions are in pixels on the garden image. Six of the nodes are
"places": the supply station the robot starts from and five plant beds it can
be sent to; the rest are plain junctions the route can pass through. The names
are simple on purpose: this is an abstract garden layout, not a real garden.

Code-level place keys stay short ASCII ids (``depot`` for the supply station,
``roses``/``tomatoes``/... for the beds). They are internal identifiers only --
the engine and the tests look them up by key. Every user-facing label comes
from :data:`LABELS`, and whether a trip carries water or fertilizer comes from
:data:`CARGO`.
"""

from __future__ import annotations

from .graph import Node, build_adj

# A small garden. The layout is a 3-row by 4-column grid of path junctions, with
# the supply station in the bottom-left and five plant beds spread around the
# edge. The topology and pixel positions are unchanged from the original map, so
# the planner and the controller behave identically; only the names differ.
NODES = [
    Node(0, 110, 500, "depot"),       # supply station: holds water + fertilizer
    Node(1, 110, 310),
    Node(2, 110, 120, "roses"),
    Node(3, 310, 120),
    Node(4, 310, 310),                 # central junction
    Node(5, 310, 500),
    Node(6, 510, 500, "lavender"),
    Node(7, 510, 310),
    Node(8, 510, 120, "tomatoes"),
    Node(9, 710, 120, "herbs"),
    Node(10, 710, 310),
    Node(11, 710, 500, "apple_tree"),
]

# Undirected garden-path segments (pairs of node ids). Unchanged topology.
EDGES = [
    (0, 1), (1, 2), (2, 3), (1, 4), (3, 4), (4, 5), (0, 5),
    (5, 6), (4, 7), (6, 7), (7, 8), (3, 8),
    (8, 9), (9, 10), (7, 10), (10, 11), (6, 11),
]

# Display order for the places (supply station first, it is the start).
PLACES = ["depot", "roses", "tomatoes", "herbs", "lavender", "apple_tree"]

# English-only display labels for every place. Rendered into the GIF, the
# canvas and the data export; nothing non-ASCII is ever drawn or written.
LABELS = {
    "depot": "Supply Station",
    "roses": "Roses",
    "tomatoes": "Tomatoes",
    "herbs": "Herbs",
    "lavender": "Lavender",
    "apple_tree": "Apple tree",
}

# What the robot carries to each plant bed: "water" for a watering run,
# "fertilizer" for a fertilizing run. Mixed across the beds so both appear.
CARGO = {
    "roses": "water",
    "tomatoes": "fertilizer",
    "herbs": "water",
    "lavender": "fertilizer",
    "apple_tree": "water",
}


def node_of(place):
    """Return the Node carrying the given place key, or None."""
    for n in NODES:
        if n.place == place:
            return n
    return None


def label_of(place):
    """The English display label for a place key (falls back to the key)."""
    return LABELS.get(place, place)


def cargo_of(place):
    """``"water"`` or ``"fertilizer"`` for a plant bed (``""`` for the station)."""
    return CARGO.get(place, "")


def adjacency():
    """Build the adjacency list for the garden paths."""
    return build_adj(NODES, EDGES)
