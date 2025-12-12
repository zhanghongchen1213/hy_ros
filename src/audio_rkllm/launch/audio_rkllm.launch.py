from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='audio_rkllm',  # 所属包名，需与 setup.py/entry_points 保持一致
            executable='audio_rkllm_node',  # 控制台脚本名，对应 setup.py 的 entry_points
            name='audio_rkllm',  # ROS 网络中的节点名称
            output='screen',  # 将节点日志输出到控制台
            emulate_tty=True,  # 提升 Python 节点的实时输出表现
            parameters=[  # ROS2 要求为 list；内部为单个 dict 容器
                {
                    # 订阅的 ASR 识别文本话题（输入），用于触发 LLM 推理
                    'asr_text_topic': '/audio/asr_text',
                    
                    # LLM 推理结果发布话题（输出）
                    'llm_result_topic': '/audio/llm_result',

                    # 语言模型文件路径 (.rkllm)
                    'llm_model_path': '/opt/rknn-llm/Qwen2-VL-2B_llm_w8a8_rk3588.rkllm',

                    # 视觉编码器模型路径 (.rknn)
                    'vision_model_path': '/opt/rknn-llm/Qwen2-VL-2B_vision_rk3588.rknn',

                    # RKLLM 推理参数配置
                    'max_new_tokens': 128,  # 最大生成 token 数
                    'max_context_len': 512, # 最大上下文长度
                    
                    # 视觉相关参数 (Qwen2-VL 需要)
                    # 对应 CLI 示例中的参数: 128 512 3 (最后一个是 core_mask?)
                    # 以及 demo.cpp 中提到的特殊 token
                    'img_start_token': '<|vision_start|>',
                    'img_end_token': '<|vision_end|>',
                    'img_pad_token': '<|image_pad|>',
                    
                    # 摄像头数据
                    'camera_flip': False,  # 图像翻转参数：True: 保持原始方向 (正)，False: 旋转180度 (反)
                }
            ],
        )
    ])
