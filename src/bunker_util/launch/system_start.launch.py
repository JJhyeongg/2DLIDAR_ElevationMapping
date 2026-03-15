from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 맵 활성화 인수 (런치 시 오버라이드 가능)
    # 예: ros2 launch bunker_util system_start.launch.py enable_map_kalman:=false
    arg_max = DeclareLaunchArgument('enable_map_max', default_value='true',
                                    description='max-z 맵 활성화')
    arg_kalman      = DeclareLaunchArgument('enable_map_kalman',      default_value='true',
                                    description='Kalman 맵 활성화')
    arg_dual_layer  = DeclareLaunchArgument('enable_map_dual_layer',  default_value='true',
                                    description='DualLayer 맵 활성화 (메인)')
    arg_csv = DeclareLaunchArgument('enable_debug_csv', default_value='false',
                                    description='CSV 디버그 로그 활성화')

    bringup = ExecuteProcess(
        cmd=['ros2', 'launch', 'bunker_sim_bringup', 'bunker_start.launch.py'],
        output='screen'
    )

    navsat = ExecuteProcess(
        cmd=['ros2', 'launch', 'bunker_nav2', 'dual_ekf_navsat.launch.py'],
        output='screen'
    )

    map_cal = Node(
        package='bunker_sim_bringup',
        executable='map_cal',
        name='map_cal',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    scan_processor = Node(
        package='bunker_util',
        executable='scan_processor',
        name='scan_processor',
        parameters=[{
            'use_sim_time': True,
            'enable_map_max':   LaunchConfiguration('enable_map_max'),
            'enable_map_kalman':      LaunchConfiguration('enable_map_kalman'),
            'enable_map_dual_layer':  LaunchConfiguration('enable_map_dual_layer'),
            'enable_debug_csv': LaunchConfiguration('enable_debug_csv'),
        }],
        output='screen'
    )

    return LaunchDescription([
        arg_max,
        arg_kalman,
        arg_dual_layer,
        arg_csv,
        bringup,
        navsat,
        map_cal,
        scan_processor,
    ])
