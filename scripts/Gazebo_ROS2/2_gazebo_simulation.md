# Gazebo simulation (robot model and controllers)

## URDF/gz_ros2_control_snippet.xacro

```xml
<!-- Fragment from the Gazebo Sim ros2_control configuration discussed in chat.
     This is NOT a complete robot URDF/Xacro file. -->
<gazebo>
  <plugin filename="libgz_ros2_control-system.so"
          name="gz_ros2_control::GazeboSimROS2ControlPlugin">
    <parameters>/home/student/ros2_ws/config/controllers.yaml</parameters>
  </plugin>
</gazebo>

```

## URDF/pedestal_snippet.xacro

```xml
<!-- Visual pedestal under the raised base (world_to_base is at z = 1.6 m).
     Paste right after the </link> that closes base_link in my_robot.urdf.xacro.
     Visual only, no collision, so the arm can never get stuck on it. -->
<link name="pedestal_link">
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
  <visual>
    <origin xyz="0 0 -0.8375"/>
    <geometry>
      <cylinder length="1.525" radius="0.08"/>
    </geometry>
    <material name="grey"/>
  </visual>
</link>
<joint name="base_to_pedestal" type="fixed">
  <parent link="base_link"/>
  <child link="pedestal_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>

```

## URDF/ros2_control_interfaces_snippet.txt

```text
The conversation established that the robot Xacro had a <ros2_control> system
using the hardware plugin:

  gz_ros2_control/GazeboSimSystem

For each of the five joints (joint1 ... joint5), the hardware block exposed:
  - command interface: position
  - state interface: position
  - state interface: velocity

The complete <ros2_control> XML block and the complete Xacro were not present
in the available conversation context, so this file records the known setup
rather than inventing the missing XML.

```

## URDF/ros2_control_system_snippet.xacro

```xml
<!-- Reconstructed fragment based on the hardware interfaces described in chat.
     Insert into the robot Xacro; this is not a complete robot description. -->
<ros2_control name="GazeboSimSystem" type="system">
  <hardware>
    <plugin>gz_ros2_control/GazeboSimSystem</plugin>
  </hardware>

  <joint name="joint1">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="joint2">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="joint3">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="joint4">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="joint5">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>

```

## ROS2/controllers.yaml

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    joint_position_controller:
      type: position_controllers/JointGroupPositionController

joint_position_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
    interface_name: position

```

## ROS2/launch_clock_bridge_node_snippet.py.txt

```text
# Node fragment discussed for the display_robot.launch.py launch file.
# This is not a complete launch file.

Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    output='screen'
),

```
