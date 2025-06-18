import os, xacro
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node
import launch_ros.descriptions
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path
from launch.conditions import IfCondition

robot_model = 'fpv_drone_ardupilot'
robot_name = 'fpv_drone_ardupilot'
pose = ['0.0', '0.0', '0.8', '1.570796327'] #Initial robot pose: x,y,z,th
world_file = 'soylu.sdf' # warehouse

def generate_launch_description():
    
    this_pkg_path = os.path.join(get_package_share_directory('fpv_drone'))

    xacro_file = os.path.join(this_pkg_path, 'urdf', robot_model+'.xacro') #.urdf

    doc = xacro.process_file(xacro_file)

    robot_desc = doc.toprettyxml(indent='  ')

    simu_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    
    # Set ign sim resource path
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(this_pkg_path, 'worlds'), ':' + str(Path(this_pkg_path).parent.resolve())
        ]
    )
    
    # 1. ArduPilot eklentisinin bulunduğu klasörün yolunu tanımlayın.
    #    os.path.expanduser('~'), '~' karakterini ev dizininize (/home/kullanici_adiniz) çevirir.
    ardupilot_plugin_dir = os.path.join(
        os.path.expanduser('~'),
        'work_workspace',
        'ardupilot_gazebo',
        'build'
    )

    # 2. SetEnvironmentVariable eylemini oluşturun.
    #    Bu yöntem, eğer bu değişken daha önceden tanımlıysa, yeni yolu eskisinin
    #    üzerine yazmak yerine başına ekler. Bu daha güvenli bir yaklaşımdır.
    gz_system_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=[
            ardupilot_plugin_dir,
            os.pathsep, # İşletim sistemine uygun ayırıcıyı ekler (Linux için ':')
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
        ]
    )

    open_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(this_pkg_path+"/rviz/fpv_drone_ardupilot.rviz")],
    )

    open_gz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
            launch_arguments=[
                ('gz_args', [this_pkg_path+"/worlds/"+world_file, ' -v 4', ' -r'])
        ]
    )
    
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-x', pose[0], '-y', pose[1], '-z', pose[2],
                   '-R', '0.0', '-P', '0.0', '-Y', pose[3],
                   '-name', robot_name,
                   '-allow_renaming', 'false'],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_name,
        output="screen",
        parameters=[{'robot_description': robot_desc,
                     'use_sim_time': True}]
    )

    """
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(this_pkg_path, 'config', 'gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )"""
    
    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[             # ign topic -t <topic_name> --info
            '/world/soylu/model/'+robot_name+'/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        parameters=[{'qos_overrides./model/'+robot_name+'.subscriber.reliability': 'reliable'}],
        output='screen',
        remappings=[            # ign topic -l
            ('/world/soylu/model/'+robot_name+'/joint_state', '/'+robot_name+'/joint_states'),
        ]
    )
    
    sitl_process = ExecuteProcess(
                cmd=[
                    'sim_vehicle.py',
                    '-v', 'ArduCopter',
                    '-f', 'gazebo-fpv',
                    '--console',
                    '--model', 'JSON',
                    '-L', 'Soylu',
                    # '-L Soylu' argümanı sürüde çakışmaya neden olacağı için
                    # kaldırıldı. 'gazebo-fpv' modeli pozisyonunu Gazebo'dan alır.
                ],
                output='screen'
            )

    return LaunchDescription(
        [
            simu_time,
            gz_resource_path,
            gz_system_plugin_path,
            open_rviz,
            open_gz,
            gz_spawn_entity,
            robot_state_publisher,
            bridge,
            sitl_process
        ]
    )