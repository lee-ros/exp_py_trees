# Experimenting with py-trees and ros2
An experimental repo for testing the py_trees library with ros2 integration

## Building and Running
To build the package run:
```bash
make build_all
```

To clean the workspace latest build run:
```bash
make clean
```

To run the POC system behaviour tree with the system state mocker, run:
```bash
make run_tree_with_state
```

## Manipulating the system state
To change the state of the mocker run:
```bash
ros2 param set /system_state_mocker system_state { state_int }
```

please take a look at the `SystemFlags` enum to see what states are supported

## Relevant Links
py-trees:
 - [Docs](https://py-trees.readthedocs.io/en/devel/index.html)
 - [GitHub](https://github.com/splintered-reality/py_trees)

</br>

py-trees-ros:
 - [Docs](https://py-trees-ros.readthedocs.io/en/devel/index.html)
 - [GitHub](https://github.com/splintered-reality/py_trees_ros)

</br>

py-trees-ros-interfaces:
 - [GitHub](https://github.com/splintered-reality/py_trees_ros_interfaces)

</br>

py-trees-ros-tutorial:
 - [Docs](https://py-trees-ros-tutorials.readthedocs.io/en/devel/index.html)
 - [GitHub](https://github.com/splintered-reality/py_trees_ros_tutorials)