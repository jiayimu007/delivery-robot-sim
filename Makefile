# Reproducible engine commands. The engine (Python) does all computation; these
# targets regenerate everything it produces. Nothing here is hand-written data.

PYTHON ?= python3

.PHONY: all data media test clean-data

# Regenerate the browser player's data, then the media previews.
all: data media

# Run the Python engine (Dijkstra + controller) and export the town graph and
# per-destination trajectories the JS player animates.
data:
	$(PYTHON) -m delivery_robot.export web/data

# Regenerate the static maps and the preview GIF from the same engine.
media:
	$(PYTHON) -m delivery_robot.render media

# The headless engine test suite (planner vs Floyd-Warshall + driving checks).
test:
	$(PYTHON) tests/run.py

clean-data:
	rm -rf web/data
