##########################################################
# LAUNCH SCRIPTS
##########################################################
.PHONY: run_tree_with_state
run_tree_with_state:
	ros2 launch src/launch/system_state_launch.py

##########################################################
# BUILD SCRIPTS
##########################################################
.PHONY: build_all
build_all:
	colcon build

.PHONY: clean
clean:
	rm -rf build/ install/ log/