import launch_ros.actions
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # base_footprint → base_link  (z=0.326m)
    tf_footprint_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_footprint',
        arguments=['0', '0', '0.326', '0', '0', '0',
                   'base_footprint', 'base_link'],
    )

    # base_link → vectornav  (IMU, 오프셋 없음)
    tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_imu',
        arguments=['0', '0', '0', '0', '0', '0',
                   'base_link', 'vectornav'],
    )

    # base_link → gps  (x=0.1567m, z=0.155m)
    tf_base_to_gps = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_gps',
        arguments=['0.1567', '0.0', '0.155', '0', '0', '0',
                   'base_link', 'gps'],
    )

    # base_link → base_lidar  (x=0.28517m, z=0.0678m, pitch=-30°, yaw=180°)
    tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_base_lidar',
        arguments=['0.28517', '0', '0.0678',
                   '3.1415926536', '-0.523598775598299', '0.0',
                   'base_link', 'base_lidar'],
    )

    return LaunchDescription([
        tf_footprint_to_base,
        tf_base_to_imu,
        tf_base_to_gps,
        tf_base_to_lidar,
    ])


if __name__ == '__main__':
    generate_launch_description()
