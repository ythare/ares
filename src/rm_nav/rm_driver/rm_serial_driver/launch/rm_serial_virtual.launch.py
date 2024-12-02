from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # 启动示例节点
        Node(
            package='rm_serial_driver',  # 替换为你的包名
            executable='virtual_serial_node',  # 替换为你的可执行文件名
            output='screen',
            respawn=True,
        ),
        # 启动 rqt 并加载参数插件
        ExecuteProcess(
            cmd=['rqt', '--standalone', 'rqt_reconfigure'],  # 替换为你想要的插件名
            output='screen'
        ),
    ])
