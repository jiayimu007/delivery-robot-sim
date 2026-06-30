# How it works

*English | [中文](how-it-works.zh-CN.md)*

The robot carries a unit of water or fertilizer from the supply station to a plant you pick. I
split it into the two parts you usually see in a mobile robot: a **planner** that decides which
way to go, and a **controller** that actually drives the route. Keeping them apart is what made
the whole thing click for me, so the code is organised the same way. The package is laid out as
`graph.py`, `town.py`, `planner.py`, `controller.py`, `simulate.py` and `render.py`.

## 1. The garden is a graph

The garden is a weighted graph (`graph.py`, `town.py`). Each junction (and each named plant, plus
the supply station) is a **node** with a position in pixels, and each path segment is an **edge**
between two nodes. The weight of an edge is just its length, computed from the two endpoints with
`math.hypot`. The garden paths are two-way, so the graph is undirected: every edge is stored in
both directions with the same weight.

That is the whole garden. Once the paths are a graph, finding a good route is a standard
shortest-path problem instead of something special to this garden.

## 2. Planning the route (Dijkstra)

Given the start (the supply station) and the goal you picked, the planner (`planner.py`) runs
**Dijkstra's algorithm** to find the shortest route.

It keeps a best-known distance to every node, starting at 0 for the supply station and infinity
for the rest. A binary heap (`heapq`) hands back the nearest node it has not finished with; it
marks that node done and checks whether going through it gives a shorter way to reach each of
its neighbours (the "relax" step), pushing the improved neighbours back onto the heap. A
`settled` flag lets it skip the stale, longer heap entries that this lazy approach leaves
behind. Because it always finishes the closest node first, the first time it finishes the
goal, that distance is final. Following the `prev` pointers back from the goal gives the
actual list of nodes to drive through.

The heap keeps the "take the nearest unfinished node" step at O(log n) per pop, which is the
textbook way to run Dijkstra. On a dozen nodes a plain linear scan would be just as fast, but
the heap is the version that also scales to a large garden.

The planner is checked against an independent **Floyd-Warshall** all-pairs solver in the
tests, on this garden and on hundreds of random graphs, so a wrong route would be caught.

## 3. Driving the route

The plan is a list of nodes. The controller (`controller.py`) drives to them one at a time.

For the junction it is currently heading for, it works out the **heading error**: the angle
between where the robot is pointing and the direction to that junction. It then biases the two
wheel speeds by that error, so one wheel runs a little faster than the other and the robot
turns to face the target. The bigger the error, the harder it turns. It also looks one
junction ahead: if a sharp turn is coming up, it eases off the throttle as it approaches, so
it can take the corner cleanly instead of overshooting. This pre-corner slowdown is what
keeps it on its path at high speed. The steering itself is plain proportional control.

The two wheel speeds become motion through a **differential-drive** model: the forward speed
is the average of the two wheels, and the turn rate is their difference divided by the wheel
base. The garden's y-axis points down (image convention), so the turn sign is set up to match
(a faster left wheel curves the robot toward its right), and there is a test that pins this
down by checking the robot's actual movement in world coordinates, not just a sign.

When the robot gets close to the junction it was aiming at, it moves on to the next one in the
plan. So the "decision" at a junction is simply: take the path toward the next node the
planner chose. The global plan picks the turns; the local controller carries them out.

## 4. The tending loop

A small state machine (`simulate.py`) ties it together. The robot starts idle at the supply
station. When you pick a plant, it picks up a unit of water or fertilizer, plans station to
plant, and drives there. On arrival it drops the unit off -- one watering or one fertilizing --
plans plant back to station, and drives home, where it goes idle again and is ready for the
next job. The block drawn on the robot shows whether it is carrying anything.

## 5. Drawing it

The renderer (`render.py`) draws the garden, the dashed planned route, the robot and the
carried unit with `Pillow`, one frame per step, and writes them out as an animated GIF. The
same code also produces the static garden images. No external animation library is involved:
`Pillow` can save a multi-frame GIF directly.

## 6. The browser player

The same engine also feeds a small browser demo, and the split there is the important part.
`export.py` runs the real engine -- it drives the tending state machine (`simulate.py`), which
plans each leg with Dijkstra and drives it with the controller -- and records the output as
JSON under `web/data/`: the garden graph (nodes with their English labels and positions, the
water/fertilizer each bed receives, and the edges), and for each plant the planned node path
for both legs plus the robot's pose (`x`, `y`, `theta`, `carrying`) for every simulation step.
Regenerate it with

```bash
python -m delivery_robot.export web/data
```

The page in `web/` (`index.html`, `player.js`, `style.css`) is **animation only**. It fetches
that JSON and draws it on a `<canvas>`: the garden paths, the dashed planned route, and the robot
moving frame by frame, with a plant selector, play/pause and a speed slider. It holds no
algorithm -- there is no Dijkstra, no steering, no kinematics in the JavaScript. Between two
recorded frames it linearly blends the position and angle so playback looks smooth, but that is
interpolation for drawing, not simulation. The speed slider only changes the playback rate; it
never re-drives the engine. So the substantive code stays in the tested Python; the browser is
just a viewer for what the engine already computed. The UI buttons can switch between English
and 中文, but the plant names drawn on the map stay English.

## 7. What this is and is not

This is a software model I wrote to study the idea. The garden is abstract and the distances are
in pixels, not metres. The planner and the controller are the standard textbook versions,
written out so I can follow every step. It is not firmware for a physical robot, and it does
not model sensors, motors or timing in any detail. The point was to understand how route
planning and path following fit together, and to be able to test that they actually work (see
[`../tests/run.py`](../tests/run.py)).
