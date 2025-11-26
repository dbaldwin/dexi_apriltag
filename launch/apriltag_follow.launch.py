from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Create the launch description
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument(
        'following_enabled',
        default_value='true',
        description='Enable AprilTag following on startup'))

    ld.add_action(DeclareLaunchArgument(
        'target_altitude',
        default_value='2.0',
        description='Target altitude above ground in meters'))

    ld.add_action(DeclareLaunchArgument(
        'proportional_gain_xy',
        default_value='0.5',
        description='Proportional gain for XY movement'))

    ld.add_action(DeclareLaunchArgument(
        'proportional_gain_z',
        default_value='0.3',
        description='Proportional gain for Z movement'))

    ld.add_action(DeclareLaunchArgument(
        'max_velocity_xy',
        default_value='0.5',
        description='Maximum XY velocity in m/s'))

    ld.add_action(DeclareLaunchArgument(
        'max_velocity_z',
        default_value='0.3',
        description='Maximum Z velocity in m/s'))

    ld.add_action(DeclareLaunchArgument(
        'deadband_pixels',
        default_value='20.0',
        description='Pixel deadband for centering'))

    ld.add_action(DeclareLaunchArgument(
        'target_tag_id',
        default_value='0',
        description='Target tag ID to follow (0 = any tag)'))

    ld.add_action(DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable visualization node'))

    # Get launch configuration values
    following_enabled = LaunchConfiguration('following_enabled')
    target_altitude = LaunchConfiguration('target_altitude')
    proportional_gain_xy = LaunchConfiguration('proportional_gain_xy')
    proportional_gain_z = LaunchConfiguration('proportional_gain_z')
    max_velocity_xy = LaunchConfiguration('max_velocity_xy')
    max_velocity_z = LaunchConfiguration('max_velocity_z')
    deadband_pixels = LaunchConfiguration('deadband_pixels')
    target_tag_id = LaunchConfiguration('target_tag_id')
    enable_visualization = LaunchConfiguration('enable_visualization')

    # AprilTag Follower Node
    apriltag_follower_node = Node(
        package='dexi_apriltag',
        executable='apriltag_follower',
        name='apriltag_follower',
        namespace='dexi',
        output='screen',
        parameters=[{
            'following_enabled': following_enabled,
            'target_altitude': target_altitude,
            'proportional_gain_xy': proportional_gain_xy,
            'proportional_gain_z': proportional_gain_z,
            'max_velocity_xy': max_velocity_xy,
            'max_velocity_z': max_velocity_z,
            'deadband_pixels': deadband_pixels,
            'target_tag_id': target_tag_id,
        }]
    )
    ld.add_action(apriltag_follower_node)

    # AprilTag Visualizer Node (optional)
    apriltag_visualizer_node = Node(
        package='dexi_apriltag',
        executable='apriltag_visualizer',
        name='apriltag_visualizer',
        namespace='dexi',
        output='screen',
        parameters=[{
            'publish_raw': False,          # Don't publish raw to save bandwidth
            'publish_compressed': True,     # Publish compressed for web viewing
        }]
    )
    # Only add if visualization is enabled
    from launch.conditions import IfCondition
    apriltag_visualizer_node = Node(
        package='dexi_apriltag',
        executable='apriltag_visualizer',
        name='apriltag_visualizer',
        namespace='dexi',
        output='screen',
        parameters=[{
            'publish_raw': False,
            'publish_compressed': True,
        }],
        condition=IfCondition(enable_visualization)
    )
    ld.add_action(apriltag_visualizer_node)

    return ld
