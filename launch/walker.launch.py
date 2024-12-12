from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    record_bag_arg = DeclareLaunchArgument(
        "record_bag", default_value="False", choices=["True", "False"],
        description="Whether to record a bag file")

    stop_args = DeclareLaunchArgument(
        "stop", default_value="False", choices=["True", "False"],
        description="Whether to stop the launch file after 30 seconds")

    # Walker node for the walker package
    walker_node = Node(
        package='walker',
        executable='walker_node',
        name='walker',
        output='screen'
    )
    # Recorder node for recording bag file if `record_bag` is True and delete the existing bag files if any and record the new bag file
    recorder_node = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("record_bag")),
        cmd=[
            "bash", "-c",
            "if [ -d 'results/bag_list/' ]; then rm -rf results/bag_list/; fi; ros2 bag record -o results/bag_list/ --all -x '/camera/*'"
        ],
    )

    # Timer action to stop the launch file after 15 seconds if `stop_args` is True
    stop_action = TimerAction(
        period=30.0,  # 30 seconds
        actions=[Shutdown()],
        condition=IfCondition(LaunchConfiguration("stop"))
    )

    return LaunchDescription([
        record_bag_arg,
        stop_args,
        walker_node,
        recorder_node,
        stop_action
    ])
