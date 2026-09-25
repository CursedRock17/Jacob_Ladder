# Build the workspace. Source ROS 2 first:
#   source /opt/ros/humble/setup.bash
#
# Targets:
#   make          format (if configured) + build
#   make build    build only
#   make format   astyle only
#   make clean    remove build/ install/ log/

# Build against the CUDA-enabled OpenCV in /usr/local rather than whichever
# OpenCV cmake happens to find first -- the JetPack image also carries a 4.8.0 in
# /usr/lib and the Ubuntu debs a 4.5.4 in /usr/lib/aarch64-linux-gnu. See the
# "OpenCV on the Jetson" section of the README. Override with:
#   make OPENCV_DIR=/some/other/lib/cmake/opencv4
OPENCV_DIR ?= /usr/local/lib/cmake/opencv4

# astyle aborts with exit 1 if the options file is missing, which would take the
# build down with it -- so formatting is skipped unless the file is present.
# Quote the patterns: unquoted, the shell expands them before astyle sees the
# `*.cpp,*.hpp` comma syntax and nothing matches.
#
# src/example_autonomous_mode is deliberately absent from this list: it carries
# its own .clang-format and is formatted with clang-format instead, so astyle
# must not touch it.
ASTYLE_OPTIONS ?= astylerc
ASTYLE_TARGETS := "src/aruco_tracker/*.cpp,*.hpp" "src/precision_land/*.cpp,*.hpp"

all: format build

format:
	@if [ -f "$(ASTYLE_OPTIONS)" ]; then \
		astyle --quiet --options=$(ASTYLE_OPTIONS) $(ASTYLE_TARGETS); \
	else \
		echo "make: skipping astyle -- '$(ASTYLE_OPTIONS)' not found"; \
	fi

build:
	@colcon build --cmake-args -DOpenCV_DIR=$(OPENCV_DIR)

clean:
	@rm -rf build install log
	@echo "All build artifacts removed"

# Lint, type-check and unit-test the mission blocks (no ROS needed). The tools
# are pinned and installed once into .check-venv, so the same versions run here
# and in CI (.github/workflows/check.yml). uv must be installed.
CHECK_VENV := .check-venv
CHECK_BIN := $(CHECK_VENV)/bin
JL_BLOCKS := src/jl_blocks
MISSIONS := $(wildcard missions/*.yaml)

$(CHECK_BIN)/pytest:
	uv venv --python 3.10 $(CHECK_VENV)
	uv pip install --python $(CHECK_VENV) ruff==0.15.20 ty==0.0.55 pytest==9.1.1 pyyaml==6.0.3

check: $(CHECK_BIN)/pytest
	$(CHECK_BIN)/ruff check $(JL_BLOCKS)
	$(CHECK_BIN)/ruff format --check $(JL_BLOCKS)
	$(CHECK_BIN)/ty check --python $(CHECK_VENV) --exclude 'src/jl_blocks/jl_blocks/ros/' $(JL_BLOCKS)/jl_blocks $(JL_BLOCKS)/test
	PYTHONPATH=$(JL_BLOCKS) $(CHECK_BIN)/pytest -q $(JL_BLOCKS)/test test/test_deploy.py
ifneq ($(MISSIONS),)
	PYTHONPATH=$(JL_BLOCKS) $(CHECK_BIN)/python -m jl_blocks.cli check $(MISSIONS)
endif

# L3: fly the jl_mission safety contract, then a mission file through the
# mission_runner, in headless SITL (spec section 8).
# Needs the jacob_ladder_sim container (./docker/run_sim_container.sh) with
# jl_mission_interfaces, jl_mission and jl_blocks built by colcon inside it.
SIM_CONTAINER ?= jacob_ladder_sim

# It restarts the container, killing anything else running in it.
sitl-test:
	docker restart $(SIM_CONTAINER) > /dev/null
	sleep 3
	docker exec --user user -w $(CURDIR) $(SIM_CONTAINER) bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && test/sitl_all.sh'

# Fly one mission file headless and check it, e.g.
#   make sitl-mission MISSION=missions/takeoff_hold_land.yaml ARGS='--expect "takeoff hold land" --finished'
# See test/sitl_mission.sh for the options. Also restarts the container.
sitl-mission:
	docker restart $(SIM_CONTAINER) > /dev/null
	sleep 3
	docker exec --user user -w $(CURDIR) $(SIM_CONTAINER) bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && test/sitl_mission.sh $(MISSION) $(ARGS)'

.PHONY: all format build clean check sitl-test sitl-mission
