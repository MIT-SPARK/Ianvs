# Ianvs

Ianvs (Janus) is a ROS2 c++ package that provides a transition path for common ROS1 design patterns that aren't as easily supported in ROS2.

This includes:
- A `NodeHandle` class that works with any ROS2 `NodeBase` implementation and provides namespaced access to underyling node
- Utility functions for handling getting single messages
- Specialized image publishers and subscribers that work with message filters

Ianvs is intended to only depend on a very select number of core ROS2 dependencies. These are currently:
- `message_filters`
- `rclcpp`
- `std_srvs`
- `sensor_msgs`

# About the Name

Ianvs is the latin spelling of Janus, the Roman deity of doorways and transitions
