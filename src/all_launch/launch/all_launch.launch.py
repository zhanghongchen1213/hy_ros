from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

from launch_ros.actions import Node

def generate_launch_description():
    # 1. uart 节点
    uart_share = get_package_share_directory('uart')
    uart_launch = os.path.join(uart_share, 'launch', 'uart.launch.py')
    
    # # 2. audio_asr 节点
    # audio_asr_share = get_package_share_directory('audio_asr')
    # audio_asr_launch = os.path.join(audio_asr_share, 'launch', 'audio_asr.launch.py')
    
    # # 3. audio_rkllm 节点
    # audio_rkllm_share = get_package_share_directory('audio_rkllm')
    # audio_rkllm_launch = os.path.join(audio_rkllm_share, 'launch', 'audio_rkllm.launch.py')
    
    # # 4. audio_tts 节点
    # audio_tts_share = get_package_share_directory('audio_tts')
    # audio_tts_launch = os.path.join(audio_tts_share, 'launch', 'audio_tts.launch.py')
    
    # 5. butter_robot_description 节点
    butter_robot_description_share = get_package_share_directory('butter_robot_description')
    butter_robot_description_launch = os.path.join(butter_robot_description_share, 'launch', 'butter_robot_description.launch.py')


    return LaunchDescription([
        # 1. uart 节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(uart_launch)
        ),
        
        # # 2. audio_asr 节点
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(audio_asr_launch)
        # ),

        # # 3. audio_rkllm 节点
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(audio_rkllm_launch)
        # ),
        
        # # 4. audio_tts 节点
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(audio_tts_launch)
        # ),
        
        # 5. butter_robot_description 节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(butter_robot_description_launch)
        ),

        # 5. foxglove_bridge 节点 (端口: 8765)
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[
                {'port': 8765},
                {'address': '0.0.0.0'},
                {'tls': False},
                {'send_buffer_limit': 10000000},
                {'use_compression': True},
                {'capabilities': ['clientPublish', 'parameters', 'parametersSubscribe', 'services', 'connectionGraph', 'assets']},
                {'asset_uri_allowlist': ['^package://(?:[-\\w]+/)*[-\\w]+\\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|tif|tiff|urdf|webp|xacro)$']}
            ]
        ),
    ])
