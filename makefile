SHELL := /bin/zsh

##########################################################
# LAUNCH SCRIPTS
##########################################################
.PHONY: run_tree_with_state
run_tree_with_state:
	ros2 launch src/launch/system_state_launch.py

.PHONY: run_ros_tree_with_state_mocker
run_ros_tree_with_state_mocker:
	ros2 launch src/launch/ros_bt_with_mocker_launch.py

##########################################################
# BUILD SCRIPTS
##########################################################
.PHONY: build_all
build_all:
	colcon build

.PHONY: clean
clean:
	rm -rf build/ install/ log/

.PHONY: source_install
source_install:
	source install/setup.zsh