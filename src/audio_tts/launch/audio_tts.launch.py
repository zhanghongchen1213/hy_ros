from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='audio_tts',
            executable='audio_tts_node',
            name='audio_tts',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'llm_topic': '/audio/llm_result',
                    'baidu_api_key': 'fUc8LIoXM4NEkJ4JLCBnTYIM',  # 需修改为个人应用的 API Key
                    'baidu_secret_key': 'e5UgkHCwyclrGziVUik2PFWbhn2eu9Hb',  # 需修改为个人应用的 Secret Key
                    'baidu_voice_id': '107192',  # 需修改为tts_api.py运行后返回的 Voice ID
                    'http_port': 8090,
                    'spd': 10, # 语速 0-15
                    'pit': 5, # 音调 0-15
                    'vol': 5, # 音量 0-15
                    'aue': 3, # 音频格式 3=mp3, 6=wav
                    'lang': 'zh', # 语言 zh=中文
                    'dialect': 'zh-CN-sichuan', # 方言，空为普通话；上海话：wuu-CN-shanghai；河南话：zh-CN-henan；四川话：zh-CN-sichuan；湖南话：zh-CN-hunan；贵州话：zh-CN-guizhou
                }
            ],
        )
    ])
