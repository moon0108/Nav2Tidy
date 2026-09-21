#!/bin/sh
# Kill every node of a running stack, leaving the zenoh router up.
#
# `pkill -f 'ros2 launch'` is not enough: it kills the launch parent and leaves
# the node processes running. A second launch then puts two of every node on the
# bus, which shows up as "more than one action server for navigate_to_pose",
# lifecycle nodes stuck in unconfigured, /clock arriving from several bridges at
# once with non-monotonic stamps, and goals that fail for no reason.
#
# Anything launched by sim.launch.py or nav2.launch.py belongs in this list.
exec pkill -9 -f 'ros2 launch|ign gazebo|ruby|map_server|planner_server|controller_server|behavior_server|bt_navigator|lifecycle_manager|parameter_bridge|robot_state_publisher|static_transform_publisher|rviz2|mppi_controller'
