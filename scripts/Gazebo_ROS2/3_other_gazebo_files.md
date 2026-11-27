# Other Gazebo files (scripts, commands, GUI)

## ROS2/start_clock_bridge.sh

```bash
#!/usr/bin/env bash
# Run this in its own terminal and leave it running while using simulation time.
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'

```

## ROS2/spawn_controllers.sh

```bash
#!/usr/bin/env bash
# Run after Gazebo and the controller_manager are up.
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

ros2 run controller_manager spawner joint_state_broadcaster
ros2 run controller_manager spawner joint_position_controller

```

## ROS2/useful_commands.md

```markdown
# ROS 2 commands from the conversation

## Check nodes and topics

```bash
ros2 node list
ros2 topic list
ros2 topic info /joint_position_controller/commands
ros2 topic echo /clock --once
```

## Send one joint position command manually

```bash
ros2 topic pub --once /joint_position_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.2, 0.0, 0.0, 0.0, 0.0]}"
```

All five values are in radians and ordered `[joint1, joint2, joint3, joint4, joint5]`.

## Inspect controller state and interfaces

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

The recorded controller state had `joint_state_broadcaster` and
`joint_position_controller` both active, with position command interfaces for
`joint1` through `joint5`.

```

## Python/robot_gui.py

```python
"""Tkinter slider GUI for the five-joint ROS 2 robot controller."""

import math
import threading
import tkinter as tk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class RobotController(Node):
    def __init__(self):
        super().__init__("robot_slider_gui")
        self.publisher = self.create_publisher(
            Float64MultiArray,
            "/joint_position_controller/commands",
            10,
        )

    def move_robot(self, angles_degrees):
        msg = Float64MultiArray()
        msg.data = [math.radians(angle) for angle in angles_degrees]
        self.publisher.publish(msg)


rclpy.init()
robot = RobotController()

spin_thread = threading.Thread(
    target=rclpy.spin,
    args=(robot,),
    daemon=True,
)
spin_thread.start()

window = tk.Tk()
window.title("Robot Joint Control")
window.geometry("480x520")

tk_label = tk.Label(
    window,
    text="Robot Joint Control",
    font=("Arial", 18, "bold"),
)
tk_label.pack(pady=12)

sliders = []

for i in range(5):
    frame = tk.Frame(window)
    frame.pack(fill="x", padx=15, pady=4)

    tk.Label(frame, text=f"Joint {i + 1}", width=9).pack(side="left")

    slider = tk.Scale(
        frame,
        from_=-180,
        to=180,
        resolution=1,
        orient="horizontal",
        length=290,
    )
    slider.set(0)
    slider.pack(side="left")

    value_label = tk.Label(frame, text="0°", width=5)
    value_label.pack(side="left")

    slider.config(
        command=lambda value, label=value_label:
        label.config(text=f"{float(value):.0f}°")
    )

    sliders.append(slider)


def send_pose():
    angles = [slider.get() for slider in sliders]
    robot.move_robot(angles)
    status.config(
        text="Command sent: " + ", ".join(f"{angle}°" for angle in angles)
    )


def reset_pose():
    for slider in sliders:
        slider.set(0)
    send_pose()


tk.Button(
    window,
    text="Move robot to selected angles",
    command=send_pose,
    font=("Arial", 11, "bold"),
).pack(pady=12)

tk.Button(
    window,
    text="Reset all joints to zero",
    command=reset_pose,
).pack(pady=3)

status = tk.Label(window, text="Adjust sliders, then move the robot.")
status.pack(pady=10)

try:
    window.mainloop()
finally:
    robot.destroy_node()
    rclpy.shutdown()

```
