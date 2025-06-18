import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def launch_setup(context, *args, **kwargs):
    """
    Bu fonksiyon, başlatma zamanında çağrılır ve başlatma argümanlarının
    değerlerine göre dinamik olarak düğümleri (node) ve işlemleri (process) oluşturur.
    """
    # Launch konfigürasyonlarını al
    swarm_arg = LaunchConfiguration('swarm')
    sensor_arg = LaunchConfiguration('sensor')
    ardupilot_arg = LaunchConfiguration('ardupilot')

    # Değerleri Python içinde kullanılabilir hale getir
    swarm_count = int(context.perform_substitution(swarm_arg))
    sensor_type = int(context.perform_substitution(sensor_arg))
    use_ardupilot = context.perform_substitution(ardupilot_arg).lower() == 'true'

    # Paket paylaşım dizinini bul
    pkg_share = get_package_share_directory('fpv_drone')
    
    # Oluşturulacak tüm eylemleri (düğümler, işlemler) tutacak liste
    launch_actions = []

    # Drone sayısını belirle (swarm=0 ise 1 drone)
    num_drones = swarm_count if swarm_count > 0 else 1

    # =================================
    # SENSÖR VE MODEL SEÇİMİ
    # =================================
    # 'sensor' parametresine göre kullanılacak XACRO dosyasını belirle.
    # NOT: Bu dosyaların urdf/models/ klasöründe bulunduğundan emin olun.
    if sensor_type == 0:
        model_filename = 'fpv_drone.urdf.xacro'
    elif sensor_type == 1:
        # TODO: 'fpv_gimbal_3d.urdf.xacro' dosyasını oluşturun
        model_filename = 'fpv_gimbal_3d.urdf.xacro'
    elif sensor_type == 2:
        # TODO: 'fpv_lidar.urdf.xacro' dosyasını oluşturun
        model_filename = 'fpv_lidar.urdf.xacro'
    else:
        # Varsayılan olarak sensörsüz modeli kullan
        model_filename = 'fpv_drone.urdf.xacro'
    
    model_path = os.path.join(pkg_share, 'urdf', 'models', model_filename)

    # Her drone için gerekli düğümleri ve işlemleri oluştur
    for i in range(num_drones):
        
        # =================================
        # NAMESPACE VE POZİSYON BELİRLEME
        # =================================
        if swarm_count > 0:
            namespace = f'fpv_{i}'
            # Dronları çarpışmamaları için farklı pozisyonlarda başlat
            x_pos, y_pos, z_pos = 0.0, float(i * 2.5), 0.75
        else:
            namespace = 'fpv_drone'
            x_pos, y_pos, z_pos = 0.0, 0.0, 0.75

        # =================================
        # ROBOT AÇIKLAMASINI OLUŞTURMA (URDF)
        # =================================
        # XACRO dosyasını işleyerek URDF içeriğini oluştur.
        # Bu URDF, her drone için özelleştirilmiş olacak (namespace vb.).
        robot_description_content = xacro.process_file(
            model_path,
            mappings={
                'namespace': namespace,
                'use_sim_time': 'true',
                # Gelecekteki ArduPilot entegrasyonu için instance numarası
                'instance': str(i) 
            }
        ).toxml()

        # =================================
        # DRONE'U GAZEBO'DA OLUŞTURMA (SPAWN)
        # =================================
        spawn_entity = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-string', robot_description_content,
                '-name', namespace,
                '-ns', namespace,
                '-allow_renaming', 'true',
                '-x', str(x_pos),
                '-y', str(y_pos),
                '-z', str(z_pos)
            ],
            output='screen'
        )
        launch_actions.append(spawn_entity)

        # =================================
        # ROBOT_STATE_PUBLISHER'I BAŞLATMA
        # =================================
        # Her drone'un kendi TF (transform) ağacını yönetmesi için.
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': True
            }]
        )
        launch_actions.append(robot_state_publisher)

        # =================================
        # ARDUPILOT SITL'İ BAŞLATMA
        # =================================
        if use_ardupilot:
            ardupilot_cmd = [
                'sim_vehicle.py',
                '-v', 'ArduCopter',
                '-f', 'gazebo-iris', # 'gazebo-fpv' modeliniz varsa onu kullanın
                '--model', 'JSON'
            ]
            if swarm_count > 0:
                # Sürü uçuşunda her drone için farklı bir instance gerekir
                ardupilot_cmd.extend(['-I', str(i)])
            else:
                # Tek drone için belirli bir lokasyonda başlat
                ardupilot_cmd.extend(['-L', 'Soylu']) # 'Soylu' lokasyonunuzun tanımlı olduğunu varsayıyoruz

            # ArduPilot'u ayrı bir terminalde çalıştır
            # Bu, ana başlatma sürecini engellemez ve logları ayırır.
            start_ardupilot_sitl = ExecuteProcess(
                cmd=['xterm', '-e'] + ardupilot_cmd,
                shell=True
            )
            launch_actions.append(start_ardupilot_sitl)

    return launch_actions


def generate_launch_description():
    """Ana başlatma fonksiyonu."""
    
    # Paket paylaşım dizinini bul
    pkg_share = get_package_share_directory('fpv_drone')
    
    # Dünya dosyasının yolunu belirle
    world_path = os.path.join(pkg_share, 'worlds', 'soylu_station.sdf')

    # ROS-Gazebo köprüsü için yapılandırma dosyasının yolu
    gz_bridge_config_path = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')

    # =================================
    # BAŞLATMA ARGÜMANLARINI TANIMLAMA
    # =================================
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

    # =================================
    # GAZEBO VE KÖPRÜYÜ BAŞLATMA
    # =================================
    # Gazebo simülasyon sunucusunu başlat
    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', world_path],
        output='screen'
    )

    # Gazebo simülasyon istemcisini (GUI) başlat
    gz_client = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen'
    )

    # ROS ve Gazebo arasındaki iletişimi sağlayan köprüyü başlat
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=['-p', gz_bridge_config_path],
        output='screen'
    )

    # =================================
    # TÜM EYLEMLERİ BİRLEŞTİRME
    # =================================
    return LaunchDescription([
        declare_swarm_arg,
        declare_sensor_arg,
        declare_ardupilot_arg,
        gz_server,
        gz_client,
        gz_bridge,
        # OpaqueFunction, diğer argümanların değerlerine göre
        # dinamik olarak eylemlerin oluşturulmasını sağlar.
        OpaqueFunction(function=launch_setup)
    ])
