from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    hiwonder_device = LaunchConfiguration("hiwonder_device")
    hiwonder_baud_rate = LaunchConfiguration("hiwonder_baud_rate")
    hiwonder_write_rate = LaunchConfiguration("hiwonder_write_rate")

    description_launch = PythonLaunchDescriptionSource(
        f"{get_package_share_directory('antsy_description')}/launch/description.launch.py")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("hiwonder_device", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("hiwonder_baud_rate", default_value="9600"),
        DeclareLaunchArgument("hiwonder_write_rate", default_value="4"),
        IncludeLaunchDescription(
            description_launch,
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f"{get_package_share_directory('antsy_control')}/launch/follow_velocity_rectangle.launch.xml"
            ),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),
        Node(
            package="hiwonder_ros2",
            executable="write_only",
            name="hiwonder_write_only",
            output="screen",
            parameters=[{
                "device": hiwonder_device,
                "baud_rate": hiwonder_baud_rate,
                "motor_read_rate": hiwonder_write_rate,
            }],
        ),
    ])
