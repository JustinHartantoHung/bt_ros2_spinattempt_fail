# BT on ROS 2 (spin modified)

Disclaimer: this is made by a complete beginner.

This is a modified version that attempts to include spin action of nav2 into the behaviour tree bt_nav_mememan_interrupt. This code unfortunately does not work, as it directly ends the behaviour tree without even returning SUCCESS or FAILURE.
For the original file from the creator, please visit: https://github.com/Adlink-ROS/BT_ros2

Also added an interrupt2 behavior tree that has 2 points to head towards, by publishing 
ros2 topic pub -1 /interrupt_event std_msgs/msg/String data:\ 'goC'
ros2 topic pub -1 /interrupt_event std_msgs/msg/String data:\ 'goD'
