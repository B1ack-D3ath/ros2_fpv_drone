import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path
import xacro
import math

def change_location(lat, lon, alt, dn, de, dd):
    """
    Başlangıç konumu ve Kuzey-Doğu-Aşağı (NED) koordinatlarındaki
    yer değiştirme bilgisiyle yeni bir coğrafi konumu hesaplar.

    Args:
        lat (float): Derece cinsinden başlangıç enlemi.
        lon (float): Derece cinsinden başlangıç boylamı.
        alt (float): Metre cinsinden başlangıç irtifası.
        dn (float): Metre cinsinden Kuzeye doğru yer değiştirme.
        de (float): Metre cinsinden Doğuya doğru yer değiştirme.
        dd (float): Metre cinsinden Aşağıya doğru yer değiştirme.

    Returns:
        tuple: Yeni enlem, boylam ve irtifayı içeren bir demet
               (yeni_enlem, yeni_boylam, yeni_irtifa).
    """
    # Dünya yarıçapı metre cinsinden (WGS-84 elipsoidi)
    R = 6378137.0

    # Enlemi radyana çevir
    lat_rad = math.radians(lat)

    # Enlemdeki değişimi hesapla
    dLat = dn / R
    yeni_lat_rad = lat_rad + dLat

    # Boylamdaki değişimi hesapla
    # Boylam için yarıçap, bulunulan enleme bağlıdır
    dLon = de / (R * math.cos(lat_rad))
    yeni_lon_rad = math.radians(lon) + dLon

    # Yeni enlem ve boylamı tekrar dereceye çevir
    yeni_enlem = math.degrees(yeni_lat_rad)
    yeni_boylam = math.degrees(yeni_lon_rad)

    # Yeni irtifayı hesapla
    # NED sisteminde 'Aşağı' pozitif olduğu için irtifadan çıkarılır.
    yeni_irtifa = alt - dd

    return (yeni_enlem, yeni_boylam, yeni_irtifa)

def launch_setup(context, *args, **kwargs):

    swarm_arg = LaunchConfiguration('swarm')
    sensor_arg = LaunchConfiguration('sensor')
    sitl_arg = LaunchConfiguration('sitl')

    swarm = int(context.perform_substitution(swarm_arg))
    sensor = int(context.perform_substitution(sensor_arg))
    sitl = context.perform_substitution(sitl_arg).lower()

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
    
    gz_pose = [0.0, 0.0, 0.1, 1.570796327]
    sitl_loc = [40.85679, 30.966991, 0.0, 0.0]
    
    gimbal_param_file_path = os.path.join(
        this_pkg_path, 'config', 'gazebo-fpv-gimbal.parm'
    )
    
    if swarm > 0:
        mav_proxy_cmd = ['mavproxy.py', '--console']
        drone_number = swarm
        
        for i in range(drone_number):
            namespace = f'fpv_{i}'
            gimbal = 'true' if sensor == 1 else 'false'
            
            mapping = {
                'namespace': namespace,
                'gimbal': gimbal,
                'sitl': sitl,
                'instance': str(i)
            }
            
            drone_desc = xacro.process_file(
                model_path,
                mappings=mapping
            ).toxml()
            
            spawn_entity = Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-string', drone_desc,
                    '-name', namespace,
                    '-ns', namespace,
                    '-allow_renaming', 'true',
                    '-x', str(i * 2.0 + gz_pose[0]), '-y', str(gz_pose[1]), '-z', str(gz_pose[2]),
                    '-R', '0.0', '-P', '0.0', '-Y', str(gz_pose[3])
                ],
                output='screen'
            )
            
            launch_actions.append(spawn_entity)
            
            robot_state_publisher = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name=f'{namespace}_state_publisher',
                namespace=namespace,
                output='screen',
                parameters=[{
                    'robot_description': drone_desc,
                    'use_sim_time': True
                }]
            )
            
            launch_actions.append(robot_state_publisher)
            
            lat, lon, alt = change_location(sitl_loc[0],
                                            sitl_loc[1],
                                            sitl_loc[2],
                                            0.0,  # North displacement
                                            i * 2.0,      # East displacement
                                            0.0)
            heading = sitl_loc[3]
            
            if sitl == 'true':
                sitl_cmd = [
                    'sim_vehicle.py',
                    '-v', 'ArduCopter',
                    '-f', 'gazebo-fpv',
                    '--model', 'JSON',
                    '-I', str(i),
                    '--console',
                    '--no-mavproxy',
                    '-l', f'{lat},{lon},{alt},{heading}'
                ]
                
                if gimbal == 'true':
                    sitl_cmd.append(f'--add-param-file={gimbal_param_file_path}')
                
                sitl_node = ExecuteProcess(
                    cmd=sitl_cmd,
                    output='screen',
                    name=f'sitl_{namespace}'
                )
                
                launch_actions.append(sitl_node)
                
                mav_proxy_cmd.extend([f'--master=tcp:127.0.0.1:{5760 * i}'])
        
        mav_proxy_node = ExecuteProcess(
            cmd=mav_proxy_cmd,
            output='screen',
            name='mavproxy'
        )
        
        launch_actions.append(mav_proxy_node)
    
    else:
        namespace = 'fpv_drone'
        gimbal = 'true' if sensor == 1 else 'false'
        
        drone_desc = xacro.process_file(
            model_path,
            mappings={
                'namespace': namespace,
                'gimbal': gimbal,
                'sitl': sitl,
                'instance': '0'
            }
        ).toprettyxml(indent="  ")
        
        debug_urdf_path = os.path.join(this_pkg_path, 'urdf', 'debug_fpv_drone_final_model.urdf')
        
        with open(debug_urdf_path, 'w') as f:
            f.write(drone_desc)
            print(f"Hata ayıklama için işlenmiş URDF dosyası şuraya kaydedildi: {debug_urdf_path}")
        
        spawn_entity = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-string', drone_desc,
                '-name', namespace,
                '-ns', namespace,
                '-allow_renaming', 'true',
                    '-x', str(gz_pose[0]), '-y', str(gz_pose[1]), '-z', '1.75',
                    '-R', '0.0', '-P', '0.0', '-Y', str(gz_pose[3])
            ],
            output='screen'
        )
        
        launch_actions.append(spawn_entity)
        
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'{namespace}_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{
                'robot_description': drone_desc,
                'use_sim_time': True
            }]
        )
        
        launch_actions.append(robot_state_publisher)
        
        lat, lon, alt = sitl_loc[0], sitl_loc[1], sitl_loc[2]
        heading = sitl_loc[3]
        
        if sitl == 'true':
            sitl_cmd = [
                'sim_vehicle.py',
                '-v', 'ArduCopter',
                '-f', 'gazebo-fpv',
                '--model', 'JSON',
                '--console',
                '-l', f'{lat},{lon},{alt},{heading}'
            ]
            
            if gimbal == 'true':
                sitl_cmd.append(f'--add-param-file={gimbal_param_file_path}')
            
            sitl_node = ExecuteProcess(
                cmd=sitl_cmd,
                output='screen',
                name=f'sitl_{namespace}'
            )
            
            launch_actions.append(sitl_node)
    
    return launch_actions


def generate_launch_description():

    declare_swarm_arg = DeclareLaunchArgument(
        'swarm', default_value='0',
        description='Number of drones to spawn. If > 0, creates a swarm.'
    )

    declare_sensor_arg = DeclareLaunchArgument(
        'sensor', default_value='0',
        description='Sensor configuration. 0: none, 1: 3D gimbal, 2: Lidar'
    )

    declare_sitl_arg = DeclareLaunchArgument(
        'sitl', default_value='false',
        description='Whether to launch ArduPilot SITL.'
    )

    return LaunchDescription([
        declare_swarm_arg,
        declare_sensor_arg,
        declare_sitl_arg,
        OpaqueFunction(function=launch_setup)
    ])