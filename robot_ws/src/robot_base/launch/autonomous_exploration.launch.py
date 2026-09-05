import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_base')
    
    # Process URDF using Command
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    robot_description = {'robot_description': Command(['xacro ', urdf_file])}

    # Config files
    nav2_yaml = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    explore_yaml = os.path.join(pkg_share, 'config', 'explore_lite.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'rviz_config.rviz')

    # 1. Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': False}]
    )

    # 2. Base Controller Node (Hardware Bridge & Odometry)
    base_controller_node = Node(
        package='robot_base',
        executable='base_controller',
        name='base_controller',
        output='screen',
        parameters=[{
            'wheel_diameter': 0.083,
            'wheel_base_width': 0.205,
            'ticks_per_rev': 340.0,
            'serial_port': '/dev/ttyTHS0',
            'baud_rate': 115200,
            'max_pwm': 255,
            'speed_to_pwm_ratio': 200.0,
        }]
    )

    # 3. RealSense Camera Node
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])
        ]),
        launch_arguments={
            'align_depth.enable': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
            'pointcloud.enable': 'false', # depthimage_to_laserscan is more efficient for 2D nav
            'depth_module.profile': '640x480x30',
            'rgb_camera.profile': '640x480x30'
        }.items()
    )

    # 4. depthimage_to_laserscan
    depth_to_laserscan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        remappings=[
            ('depth', '/camera/aligned_depth_to_color/image_raw'),
            ('depth_camera_info', '/camera/color/camera_info'),
            ('scan', '/scan')
        ],
        parameters=[{
            'scan_time': 0.033,
            'range_min': 0.25,
            'range_max': 4.0,
            'scan_height': 5, # 5 rows of pixels used for the scan
            'output_frame': 'camera_link'
        }]
    )

    # 5. RTAB-Map SLAM Node
    rtabmap_node = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'subscribe_depth': True,
            'subscribe_scan': True,
            'subscribe_odom_info': False,
            'approx_sync': True,
            'visual_odometry': False, # Using Wheel Odometry
            'odom_frame_id': 'odom',
            'database_path': '~/.ros/rtabmap.db',
            'Rtabmap/TimeThr': '700',
            'RGBD/LinearUpdate': '0.05',
            'RGBD/AngularUpdate': '0.05',
            'Grid/RangeMax': '4.0',
            'Mem/UseOdomGravity': 'true',
            'Optimizer/GravitySigma': '0.3',
            'Reg/Strategy': '0', # Visual registration
            'Reg/Force3DoF': 'true', # 2D mapping for Nav2
        }],
        remappings=[
            ('odom', '/odom'),
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
            ('scan', '/scan')
        ],
        arguments=['--delete_db_on_start']
    )

    # 6. Nav2 Bringup
    nav2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': 'False',
            'params_file': nav2_yaml
        }.items()
    )

    # 7. Explore Lite
    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[explore_yaml]
    )

    # 8. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        base_controller_node,
        realsense_node,
        depth_to_laserscan_node,
        rtabmap_node,
        nav2_node,
        explore_node,
        rviz_node
    ])
