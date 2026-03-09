import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # 获取 'rm_behavior_tree' 包的共享目录路径
    bt_config_dir = os.path.join(get_package_share_directory('rm_behavior_tree'), 'config')
    
    # 定义启动配置参数，默认为 'full' 和 'False'
    style = LaunchConfiguration('style', default='full')
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    # 使用 PathJoinSubstitution 拼接路径，生成 XML 配置文件的路径
    bt_xml_dir = PathJoinSubstitution([bt_config_dir, style]), ".xml"

    # 定义一个 'rm_behavior_tree' 节点的配置
    rm_behavior_tree_node = Node(
        package='rm_behavior_tree',          # 节点所属的包
        executable='rm_behavior_tree',       # 要执行的节点程序
        respawn=True,                        # 如果节点崩溃，自动重启
        respawn_delay=3,                     # 崩溃后自动重启的延迟时间（秒）
        parameters=[
            {
              'style': bt_xml_dir,           # 配置文件的路径
              'use_sim_time': use_sim_time,  # 是否使用模拟时间
            }
        ]
    )

    # 返回一个 LaunchDescription 对象，该对象包含了需要启动的节点
    return LaunchDescription([rm_behavior_tree_node])