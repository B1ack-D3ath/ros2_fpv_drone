import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path
import xacro

def launch_setup(context, *args, **kwargs):

    swarm_arg = LaunchConfiguration('swarm')
    sensor_arg = LaunchConfiguration('sensor')
    sitl_arg = LaunchConfiguration('sitl')

    swarm = int(context.perform_substitution(swarm_arg))
    sensor = int(context.perform_substitution(sensor_arg))
    sitl = context.perform_substitution(sitl_arg).lower() == 'true'

    this_pkg_path = get_package_share_directory('fpv_drone')
    
    model_path = os.path.join(this_pkg_path, 'urdf', 'fpv_drone.xacro')
    
    launch_actions = []
    
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(this_pkg_path, 'worlds'), ':' + str(Path(this_pkg_path).parent.resolve())
        ]
    )
    
    launch_actions.append(set_gz_resource_path)
    
    gz_plugin_path = os.path.join(
        os.path.expanduser('~'),
        'work_workspace',
        'ardupilot_gazebo',
        'build'
    )
    
    set_gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=[
            gz_plugin_path,
            os.pathsep,
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
        ]
    )
    
    launch_actions.append(set_gz_plugin_path)
    
    if swarm > 0:
        world_file_name = 'soylu.sdf'
    
    else:
        world_file_name = 'soylu_station.sdf'
    
    open_gz = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r', world_file_name],
        output='screen'
    )
    
    launch_actions.append(open_gz)

    open_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )
    
    if swarm > 0:
        open_rviz.arguments = ['-d', os.path.join(this_pkg_path, 'rviz', 'fpv_swarm.rviz')]
    else:
        open_rviz.arguments = ['-d', os.path.join(this_pkg_path, 'rviz', 'fpv_drone.rviz')]

    launch_actions.append(open_rviz)
    
    if swarm > 0:
        for i in range(swarm):
            if sensor == 0:
                pass



def generate_launch_description():

    declare_swarm_arg = DeclareLaunchArgument(
        'swarm', default_value='0',
        description='Number of drones to spawn. If > 0, creates a swarm.'
    )

    declare_sensor_arg = DeclareLaunchArgument(
        'sensor', default_value='0',
        description='Sensor configuration. 0: none, 1: 3D gimbal, 2: Lidar'
    )

    declare_ardupilot_arg = DeclareLaunchArgument(
        'ardupilot', default_value='false',
        description='Whether to launch ArduPilot SITL.'
    )

    return LaunchDescription([
        declare_swarm_arg,
        declare_sensor_arg,
        declare_ardupilot_arg,
        OpaqueFunction(function=launch_setup)
    ])