from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='audio_tts',  # 所属包名，需与 setup.py/entry_points 保持一致
            executable='audio_tts_node',  # 控制台脚本名，对应 setup.py 的 entry_points
            name='audio_tts',  # ROS 网络中的节点名称
            output='screen',  # 将节点日志输出到控制台
            emulate_tty=True,  # 提升 Python 节点的实时输出表现
            parameters=[  # ROS2 要求为 list；内部为单个 dict 容器
                {
                    'llm_result_topic': '/audio/llm_result',
                }
            ],
        )
    ])
