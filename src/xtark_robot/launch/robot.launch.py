from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xtark_robot',
            executable='xtark_robot_node',
            # name='xtark_robot',
            output='screen',
            parameters=[
                {'base_frame': 'base_footprint'},
                {'odom_frame': 'odom'},
                {'imu_frame': 'imu_link'},
                {'odom_topic': 'odom'},
                {'imu_topic': 'imu'},
                {'battery_topic': 'bat_vol'},
                {'cmd_vel_topic': 'cmd_vel'},
                {'robot_port': '/dev/ttyACM0'},
                {'robot_port_baud': 230400},
                {'pub_odom_tf': True},
                {'light_mode': 2},
                {'rgb_r': 0},
                {'rgb_g': 255},
                {'rgb_b': 0},
                {'akm_servo_offset': 120}
            ]
        )
    ])