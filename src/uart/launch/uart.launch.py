from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uart',  # 您的包名
            executable='uart_node',  # 在 setup.py 中 entry_points 定义的可执行文件名
            name='uart',  # ROS 网络中节点的自定义名称
            output='screen',  # 将节点输出打印到屏幕
            emulate_tty=True,  # 对于Python节点，这通常能帮助立即看到 print 输出
            parameters=[  # 如果需要传递参数 (可选)
                # {'param_name': 'param_value'}
            ]
        )
    ])
