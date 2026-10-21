# MPU6050 on the Jetson's I2C bus (not the base MCU). Publishes imu/data only.
#
#   ros2 launch mpu6050_imu imu.launch.py i2c_bus:=7 i2c_addr:=104
#
# i2c_addr is DECIMAL (104 == 0x68).

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('i2c_bus', default_value='7'),
        DeclareLaunchArgument('i2c_addr', default_value='104'),
        DeclareLaunchArgument('frame_id', default_value='imu_link'),
        DeclareLaunchArgument('rate_hz', default_value='100.0'),
        # Accepted for compatibility with robot.launch.py / slam_imu.sh. This
        # driver never publishes TF; the EKF (or rf2o) owns odom->base_footprint.
        DeclareLaunchArgument('publish_odom_tf', default_value='false'),

        Node(
            package='mpu6050_imu',
            executable='imu_node',
            name='mpu6050_imu',
            output='screen',
            parameters=[{
                'i2c_bus': ParameterValue(LaunchConfiguration('i2c_bus'), value_type=int),
                'i2c_addr': ParameterValue(LaunchConfiguration('i2c_addr'), value_type=int),
                'frame_id': LaunchConfiguration('frame_id'),
                'rate_hz': ParameterValue(LaunchConfiguration('rate_hz'), value_type=float),
                'publish_odom_tf': ParameterValue(
                    LaunchConfiguration('publish_odom_tf'), value_type=bool),
            }],
        ),
    ])
