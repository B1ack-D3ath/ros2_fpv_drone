import os, xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction, ExecuteProcess
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path


def launch_opaque_function(context, *args, **kwargs):
    pose = ['0.0', '0.0', '0.75', '1.570796327'] #Initial robot pose: x,y,z,th
    world_file = 'soylu_station.sdf' # warehouse
    
    this_pkg_path = os.path.join(get_package_share_directory('fpv_drone'))
    
    model = LaunchConfiguration('model').perform(context)  # Get the model name from launch configuration
    
    test_model = f'test_{model}'  # e.g. test_fpv_base, test_fpv_cam, etc.

    test_model_xacro_file = os.path.join(this_pkg_path, 'urdf', test_model+'.xacro') #.urdf

    test_model_doc = xacro.process_file(test_model_xacro_file)

    test_model_desc = test_model_doc.toprettyxml(indent='  ')
    debug_urdf_path = os.path.join(this_pkg_path, 'urdf', f'debug_{test_model}.urdf')
    
    run_list = []
    
    with open(debug_urdf_path, 'w') as f:
        f.write(test_model_desc)
        print(f"Hata ayıklama için işlenmiş URDF dosyası şuraya kaydedildi: {debug_urdf_path}")

    simu_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    
    run_list.append(simu_time)
    
    # Set ign sim resource path
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(this_pkg_path, 'worlds'), ':' + str(Path(this_pkg_path).parent.resolve())
        ]
    )
    
    run_list.append(gz_resource_path)

    open_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(this_pkg_path + f"/rviz/{test_model}.rviz")],
    )
    
    run_list.append(open_rviz)

    open_gz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
            launch_arguments=[
                ('gz_args', [this_pkg_path+"/worlds/"+world_file, ' -v 4', ' -r'])
        ]
    )
    
    run_list.append(open_gz)
    
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', test_model_desc,
                '-x', pose[0], '-y', pose[1], '-z', pose[2],
                '-R', '0.0', '-P', '0.0', '-Y', pose[3],
                '-name', test_model,
                '-allow_renaming', 'false'],
    )
    
    run_list.append(gz_spawn_entity)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=test_model,
        output="screen",
        parameters=[{'robot_description': test_model_desc,
                    'use_sim_time': True}]
    )
    
    run_list.append(robot_state_publisher)
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/soylu/model/'+test_model+'/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        parameters=[{'qos_overrides./model/'+test_model+'.subscriber.reliability': 'reliable'}],
        output='screen',
        remappings=[
            ('/world/soylu/model/'+test_model+'/joint_state', '/'+test_model+'/joint_states'),
        ]
    )
    
    run_list.append(bridge)
    
    if model == 'fpv_ardupilot':
        sitl_process = ExecuteProcess(
                    cmd=[
                        'sim_vehicle.py',
                        '-v', 'ArduCopter',
                        '-f', 'gazebo-fpv',
                        '--console',
                        '--model', 'JSON',
                        '-L', 'Soylu',
                    ],
                    output='screen'
                )
        run_list.append(sitl_process)

    return run_list    

def generate_launch_description():
    
    declare_model = DeclareLaunchArgument(
        'model',
        default_value='fpv_drone',
        description='Robot model to launch, e.g. fpv_base, fpv_cam, fpv_motor, gimbal_3d')
    
    opaque_function = OpaqueFunction(function=launch_opaque_function)

    return LaunchDescription(
        [
            declare_model,
            opaque_function
        ]
    )