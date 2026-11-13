ssh:
cd ~/ros2_ws

colcon build --packages-up-to <your_package_name>

source install/setup.bash

ros2 run <your_package_name> autonomous_driving

another terminal:
ros2 topic pub --once /autonomous_command interfaces/msg/AutonomousCommand "{move: 1}"

arreter:
ros2 topic pub --once /autonomous_command interfaces/msg/AutonomousCommand "{move: 0}"