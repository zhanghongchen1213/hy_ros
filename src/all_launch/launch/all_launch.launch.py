from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # cerebellum 功能包：示例聚合项
    # cerebellum_share = get_package_share_directory('cerebellum')
    # cerebellum_launch = os.path.join(cerebellum_share, 'launch', 'cerebellum.launch.py')
    
    uart_share = get_package_share_directory('uart')
    uart_launch = os.path.join(uart_share, 'launch', 'uart.launch.py')
    
    audio_asr_share = get_package_share_directory('audio_asr')
    audio_asr_launch = os.path.join(audio_asr_share, 'launch', 'audio_asr.launch.py')

    return LaunchDescription([
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(cerebellum_launch)
        # ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(uart_launch)
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(audio_asr_launch)
        ),
    ])
