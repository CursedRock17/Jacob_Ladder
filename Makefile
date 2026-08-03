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

.PHONY: all format build clean
