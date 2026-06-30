# Garden Tending Robot

*English | [中文](README.zh-CN.md)*

[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

A small robot that keeps a garden's plants watered and fertilized. You pick a plant, it plans
the shortest route from the supply station, drives there along the garden paths, drops off a
unit of water or fertilizer, and comes back. I wrote it to understand how the two halves of a
tending robot fit together: a planner that chooses the route, and a controller that follows
it. It is a software model, not firmware for a real robot.

![The garden map](media/map-overview.png)

## How it works

The garden paths are a weighted graph: junctions are nodes, path segments are edges, and each
edge weight is its length. **Dijkstra's algorithm** finds the shortest route from the supply
station to the plant you picked. The robot then drives that route one junction at a time,
steering toward the next one with a simple proportional controller and a differential-drive
motion model. It eases off the throttle before a corner so it stays on the path. At each
junction the "decision" is just to take the path toward the next node in the plan, so the
global plan picks the turns and the local controller carries them out.

There is a longer write-up in [docs/how-it-works.md](docs/how-it-works.md).

## The simulation

It is pure Python. `numpy` handles the math and `Pillow` draws the frames, so it runs from
the command line. Running it produces an animated GIF of a tending run: the
dashed blue line is the planned route, the yellow block on the robot is the water or
fertilizer it carries, and the status line shows what it is doing.

![A tending run](media/delivery.gif)

The robot picks up a unit at the supply station, carries it to the plant, drops it off to
complete one watering or one fertilizing, and returns empty before it is ready for the next
job.

To regenerate the GIF and the static maps:

```bash
python -m delivery_robot.render media
```

## The browser demo

There is also a small browser player in [`web/`](web/). It is animation only: the Python
engine plans each tending run with Dijkstra, drives it with the controller, and writes the
result out as JSON (the garden graph, the planned node path for each leg, and the robot's pose
for every frame). The page loads that data and replays it on a `<canvas>` with a plant
selector, play/pause, a speed slider and a status line. It does not re-plan or re-simulate
anything in the browser; it only draws the frames the engine already computed. The player is
plain JavaScript and Canvas with no libraries.

Generate the data the engine exports:

```bash
python -m delivery_robot.export web/data   # or: make data
```

Then serve the folder (it is a static site) and open it:

```bash
python -m http.server   # then visit http://localhost:8000/  (redirects to web/)
```

The UI buttons have an English / 中文 toggle; the plant names drawn on the map stay English.

## Running the tests

The planner and the driving logic run headlessly, with no rendering:

```bash
python tests/run.py
# or, if pytest is installed:
pytest tests/run.py
```

The tests compare Dijkstra against an independent Floyd-Warshall solver (on this garden and on
hundreds of seeded random graphs, including disconnected ones), check that every route it
returns is a real path of the reported length, and drive the robot to every plant across the
whole speed range to confirm it stays on the path and arrives.

## Layout

```
delivery_robot/   the simulation package (graph, planner, controller, town, simulate, render, export)
tests/            the headless test suite (run.py)
docs/             a longer explanation of how it works
media/            the GIF and the static maps used here
web/              the browser player (index.html, player.js, style.css) + exported data/
```

## Requirements

Python 3.9+ with `numpy` and `Pillow`.

## License

MIT, see [LICENSE](LICENSE). Made by Jiayi Mu ([github.com/jiayimu007](https://github.com/jiayimu007)).
