from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_vision_arg = DeclareLaunchArgument(
        'enable_vision',
        default_value='true',
        description='Launch pose_bridge_node and motion_detector_node',
    )
    enable_web_video_arg = DeclareLaunchArgument(
        'enable_web_video',
        default_value='false',
        description='Launch web_video_server for remote MJPEG debugging',
    )
    trt_host_arg = DeclareLaunchArgument(
        'trt_host',
        default_value='127.0.0.1',
        description='Host address of trt_pose_server running on Jetson',
    )
    trt_port_arg = DeclareLaunchArgument(
        'trt_port',
        default_value='5555',
        description='TCP port of trt_pose_server',
    )

    enable_vision = LaunchConfiguration('enable_vision')
    enable_web_video = LaunchConfiguration('enable_web_video')
    trt_host = LaunchConfiguration('trt_host')
    trt_port = LaunchConfiguration('trt_port')

    lidar_node = Node(
        name='rplidar_composition',
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            # 'serial_baudrate': 115200,  # A1 / A2
            'serial_baudrate': 256000,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }],
        emulate_tty=True,
    )

    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        parameters=[{
            'video_device': '/dev/video0',
            'image_width': 320,
            'image_height': 240,
            'framerate': 10.0,
            'pixel_format': 'yuyv',
        }],
        emulate_tty=True,
    )

    system_check_ack_node = Node(
        package='system_check_ack',
        executable='system_check_ack_node',
        emulate_tty=True,
    )

    pose_bridge_node = Node(
        package='pose_vision',
        executable='pose_bridge_node',
        output='screen',
        condition=IfCondition(enable_vision),
        parameters=[{
            'trt_host': trt_host,
            'trt_port': trt_port,
            'image_topic': '/image_raw',
            'pose_image_topic': '/vision/image_pose',
            'keypoints_topic': '/vision/pose_keypoints',
        }],
        emulate_tty=True,
    )

    motion_detector_node = Node(
        package='pose_vision',
        executable='motion_detector_node',
        output='screen',
        condition=IfCondition(enable_vision),
        parameters=[{
            'keypoints_topic': '/vision/pose_keypoints',
            'motion_topic': '/vision/detected_motion',
            'reset_service': '/vision/reset_67',
        }],
        emulate_tty=True,
    )

    web_video_node = Node(
        package='web_video_server',
        executable='web_video_server',
        condition=IfCondition(enable_web_video),
        emulate_tty=True,
    )

    ld = LaunchDescription([
        enable_vision_arg,
        enable_web_video_arg,
        trt_host_arg,
        trt_port_arg,
        lidar_node,
        camera_node,
        system_check_ack_node,
        pose_bridge_node,
        motion_detector_node,
        web_video_node,
    ])
    return ld
