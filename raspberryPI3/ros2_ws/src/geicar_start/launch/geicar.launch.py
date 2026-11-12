from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    joystick_node = Node(
        package="joystick",
        executable="joystick_ros2.py",
        emulate_tty=True
    )

    joystick_to_cmd_node = Node(
        package="joystick",
        executable="joystick_to_cmd",
        emulate_tty=True
    )

    can_rx_node = Node(
        package="can",
        executable="can_rx_node",
        emulate_tty=True
    )

    can_tx_node = Node(
        package="can",
        executable="can_tx_node",
        emulate_tty=True
    )

    car_control_node = Node(
        package="car_control",
        executable="car_control_node",
        emulate_tty=True
    )

    safety_stop_node = Node(
        package="safety_stop",
        executable="safety_stop_node",
        parameters=[{
            "stop_dist_front_cm": 30,
            "stop_dist_rear_cm": 30,
            "slow_dist_front_cm": 55,
            "slow_dist_rear_cm": 55,
            "us_timeout_ms": 99999999999,
            "cmd_timeout_ms": 999,
            "log_actions": True
        }],
        emulate_tty=True
    )

    obstacle_stop_node = Node(
        package="obstacle_stop",
        executable="obstacle_stop_node",
        output="screen",
        emulate_tty=True
    )


    config_dir = os.path.join(get_package_share_directory('imu_filter_madgwick'), 'config')

    imu_filter_madgwick_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        parameters=[os.path.join(config_dir, 'imu_filter.yaml')],
        emulate_tty=True
    )


    system_check_node = Node(
        package="system_check",
        executable="system_check_node",
        emulate_tty=True
    )


    ld.add_action(joystick_node)
    ld.add_action(joystick_to_cmd_node)
    ld.add_action(can_rx_node)
    ld.add_action(can_tx_node)
    ld.add_action(car_control_node)
    ld.add_action(safety_stop_node)
    #ld.add_action(obstacle_stop_node)
    ld.add_action(imu_filter_madgwick_node)
    ld.add_action(system_check_node)

    return ld
