from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 1. uart 节点
    uart_share = get_package_share_directory('uart')
    uart_launch = os.path.join(uart_share, 'launch', 'uart.launch.py')
    
    # 2. audio_asr 节点
    audio_asr_share = get_package_share_directory('audio_asr')
    audio_asr_launch = os.path.join(audio_asr_share, 'launch', 'audio_asr.launch.py')
    
    # 3. audio_rkllm 节点
    audio_rkllm_share = get_package_share_directory('audio_rkllm')
    audio_rkllm_launch = os.path.join(audio_rkllm_share, 'launch', 'audio_rkllm.launch.py')

    return LaunchDescription([
        # 1. uart 节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(uart_launch)
        ),
        
        # 2. audio_asr 节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(audio_asr_launch)
        ),

        # 3. audio_rkllm 节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(audio_rkllm_launch)
        ),
    ])
