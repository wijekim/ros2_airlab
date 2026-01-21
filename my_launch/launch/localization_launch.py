import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # 1. 패키지의 share 디렉토리 경로 찾기
    package_dir = get_package_share_directory('my_launch')
    
    # 2. 맵 파일 경로 설정 (maps 폴더 안에 있을 경우)
    map_yaml_file = os.path.join(package_dir, 'maps', 'map_1767696184.yaml')

    # 3. Map Server 노드
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': map_yaml_file}]
    )

    # 4. AMCL 노드
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[{'set_initial_pose': True}]
    )

    # 5. Dynamixel Operators (별도 xterm 창에서 실행)
    operator_node = Node(
        package='dynamixel_workbench_operators',
        executable='dynamixel_workbench_operators',
        name='dynamixel_workbench_operators',
        prefix='xterm -e',  # 이 설정으로 새 창이 뜹니다.
        output='screen'
    )

    # 6. Lifecycle 제어 (Nav2 노드 활성화)
    map_server_activate = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_util', 'lifecycle_bringup', 'map_server'],
        output='screen'
    )

    amcl_activate = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_util', 'lifecycle_bringup', 'amcl'],
        output='screen'
    )

    return LaunchDescription([
        map_server_node,
        amcl_node,
        operator_node,      # 추가된 조종 노드
        map_server_activate,
        amcl_activate
    ])