# The robor_controllers package

This package provides various motion controllers for the Robor project. All of the controllers publish messages of type `geometry_msgs/Twist` under topic `controls`. They all work in a constant-rate loop, where the rate can be set via parameters. Also all of the controllers are provided as regular nodes as well as nodelets.

## The manual_controller

Manual controller subscribes to messages of type `sensor_msgs/Joy` from topic `joy` as well as messages of type `geometry_msgs/Twist` from topic `keys`. It converts these messages into appropriate control signals. The main loop starts when the node is activated and one or both sources of input messages are set to true. Additionally, the node provides a functionallity of a first-order, linear smoothing filter for the input signals.

To run manual controller start node `manual_controller_node` or nodelet `robor_controllers/ManualController`. Use `Manual Controller` Rviz panel for convenient usage.

### Parameters:
- `active` (`bool`, default: `false`) - activates the node,
- `publish_ref_vel` (`bool`, default: `false`) - if set to true, the node publishes messages of type `nav_msgs/Odometry` under topic `reference_state` instead of control signals (after checking setting this parameter to true the node must be restarted (deactivate-activate),
- `use_joy` (`bool`, default: `false`) - use signals from joystick as input,
- `use_keys` (`bool`, default: `false`) - use signals from keyboard as input,
- `loop_rate` (`double`, default: `100.0`) - the rate of the main loop in hertz,
- `time_constant` (`double`, defaul: `0.0`) - the time constant of a smoothing filter in seconds,
- `linear_gain` (`double`, default: `0.3`) - the gain of the controller for the control signals related to linear motion (in meters per second),
- `angular_gain` (`double`, default: `0.5`) - the gain of the controller for the control signals related to rotation (in radians per second),
- `parent_frame_id` (`string`, default: `odom`) - the parent frame id for the `reference_state` messages,
- `child_frame_id` (`string`, default: `reference`) - the child frame id for the `reference_state` messages.

## The feedback_controller

Feedback controller is a set of three simple P-type controllers with feedforward action applied for each degree of freedom. The controller subscribes to messages of type `nav_msgs/Odometry` from topic `reference_state` as well as messages of the same type from topic `robot_state` to calculate appropriate control signals. The controller also provides a scaling procedure to preserve the orientation of control vector in case of saturation.

To run feedback controller start node `feedback_controller_node` or nodelet `robor_controllers/FeedbackController`. Use `Feedback Controller` Rviz panel for convenient usage.

### Parameters:
- `active` (`bool`, default: `false`) - activates the node,
- `run` (`bool`, default: `false`) - activates the main loop and control signals publishing,
- `use_ff` (`bool`, default: `true`) - use the feedforward action,
- `loop_rate` (`double`, default: `100.0`) - the rate of the main loop in hertz,
- `gain_x` (`double`, default: `0.5`) - the gain of the controller for longitudinal motion (in meters per second per meter),
- `gain_y` (`double`, default: `0.5`) - the gain of the controller for tranversal motion (in meters per second per meter),
- `gain_theta` (`double`, default: `0.5`) - the gain of the controller for angular motion (in radians per second per radian),
- `max_u` (`double`, default: `0.5`) - the saturation level for longitudinal velocity (in meters per second),
- `max_v` (`double`, default: `0.5`) - the saturation level for transversal velocity (in meters per second),
- `max_w` (`double`, default: `0.5`) - the saturation level for angular velocity (in radians per second).

## The potentials_controller

Potentials controller is an advanced motion controller incorporating obstacle avoidance. The controller subscribes to messages of type `nav_msgs/Odometry` from topic `reference_state` and messages of the same type from topic `robot_state` as well as messages of type `obstacle_detection/Obstacles` from topic `obstacles`. T.B.C.
