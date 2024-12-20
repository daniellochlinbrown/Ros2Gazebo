from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, Command
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    # Declare environment variables and launch arguments
    ld = LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='serf',
            description='Name of the world to load'
        ),
        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            description='Enable verbose output'
        ),
        SetEnvironmentVariable(
            name='PX4_GZ_STANDALONE',
            value='1'
        ),
        SetEnvironmentVariable(
            name='PX4_SYS_AUTOSTART',
            value='4001'
        ),
        SetEnvironmentVariable(
            name='PX4_SIM_MODEL',
            value='gz_x500'
        ),
        SetEnvironmentVariable(
            name='PX4_GZ_WORLD',
            value=LaunchConfiguration('world')
        )
    ])

    # Using FindPackage to get the correct package path
    package_path = FindPackageShare(package='uas_gazebo_sim')

    # Function to conditionally configure commands
    def launch_setup(context, *args, **kwargs):
        verbose_flag = [
            '-v'] if context.launch_configurations['verbose'] == 'true' else []

        start_gazebo = ExecuteProcess(
            cmd=[
                'gz', 
                'sim', *verbose_flag, '-r',
                Command([package_path,'/worlds/',
                        LaunchConfiguration('world'), '/', LaunchConfiguration('world'), '.sdf'])
            ],
            output='screen',
            shell=True
        )

        start_px4_sitl = ExecuteProcess(
            cmd=[
                'bash', '-c',
                '$PX4_DIR/build/px4_sitl_default/bin/px4'
            ],
            output='screen',
            shell=True,
            additional_env={
                'PX4_GZ_STANDALONE': '1',
                'PX4_SYS_AUTOSTART': '4001',
                'PX4_SIM_MODEL': 'gz_x500',
                'PX4_GZ_WORLD': LaunchConfiguration('world')
            }
        )

        return [start_gazebo, start_px4_sitl]

    # Add function to launch description
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
