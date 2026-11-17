ssh:
cd ~/ros2_ws

colcon build --packages-up-to <your_package_name>

source install/setup.bash


J'arrive pas avec geivar.launch.py... Faut demander à Raph
ros2 run can can_rx_node
ros2 run can can_tx_node
ros2 run obstacle_stop obstacle_stop_node
ros2 run autonomous_driving autonomous_driving_node

Pour montrer que le ack passe:
ros2 topic echo /movement_ack
