"""
音频流式识别节点（audio_asr).

本模块实现基于 sherpa-onnx 的流式 ASR 能力，遵循项目规则：
- 所有运行参数通过 launch 文件传入，源码不设默认值；缺失即报错退出。
- 话题名、模型路径、设备名、端点检测参数均参数化，禁止硬编码。

节点行为：
- 订阅控制话题（Bool）以触发识别开始；关闭时继续至端点检测完成再退出。
- 根据 `use_cli` 选择 CLI 或 Python API 两种实现路径。
- 将识别文本发布到配置的话题（String）。
"""

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import threading
import time
import subprocess
import shlex
import shutil

try:
    import sounddevice as sd
except (ImportError, OSError) as e:
    print(f"FATAL: 无法导入 sounddevice 模块或缺少 PortAudio 库: {e}", file=sys.stderr)
    print("请执行以下命令安装依赖:\n  sudo apt-get install libportaudio2", file=sys.stderr)
    sys.exit(1)

try:
    import sherpa_onnx
except (ImportError, OSError) as e:
    print(f"FATAL: 无法导入 sherpa_onnx 模块: {e}", file=sys.stderr)
    print("请重新尝试安装 sherpa-onnx 模块！", file=sys.stderr)
    sys.exit(1)


class AudioAsrNode(Node):
    def __init__(self):
        """
        构造函数.

        用途：
        - 声明并读取所有必要参数（严格校验类型与非空），创建发布/订阅器。
        - 初始化识别状态与线程句柄，输出启动日志。

        读取的关键参数（均由 launch 传入）：
        - chat_topic：控制识别开关的订阅话题（Bool）
        - asr_text_topic：识别文本发布话题（String）
        - provider：推理后端提供者（例如 'rknn'）
        - encoder/decoder/joiner/tokens：模型与词表路径
        - alsa_device：ALSA 输入设备标识
        - silence_timeout_sec：CLI 模式下的静默超时（秒）
        - use_cli：是否使用 CLI 模式（True 为 CLI，False 为 Python API）
        - 端点检测参数：rule1_min_trailing_silence / rule2_min_trailing_silence / rule3_max_segment_length
        """
        super().__init__(
            'audio_asr',
            allow_undeclared_parameters=False,
            automatically_declare_parameters_from_overrides=True,
        )
        chat_topic = self._require_str('chat_topic')
        asr_text_topic = self._require_str('asr_text_topic')
        self.provider = self._require_str('provider')
        self.encoder = self._require_str('encoder')
        self.decoder = self._require_str('decoder')
        self.joiner = self._require_str('joiner')
        self.tokens = self._require_str('tokens')
        self.alsa_device = self._require_str('alsa_device')
        self.silence_timeout_sec = self._require_float('silence_timeout_sec')
        self.use_cli = bool(self.get_parameter('use_cli').value)
        self.rule1_min_trailing_silence = self._require_float('rule1_min_trailing_silence')
        self.rule2_min_trailing_silence = self._require_float('rule2_min_trailing_silence')
        self.rule3_min_utterance_length = self._require_float('rule3_min_utterance_length')
        self.target_device_idx = self._require_int('target_device_idx')

        self.pub_text = self.create_publisher(String, asr_text_topic, 10)
        self.sub_chat = self.create_subscription(Bool, chat_topic, self._on_chat, 10)

        self._recognizing = False
        self._rec_thread = None
        self._chat_last = False
        self.get_logger().info('音频ASR节点已启动')

    def _require_str(self, name: str) -> str:
        """
        读取字符串参数（严格校验).

        参数：
        - name：参数名

        返回：
        - 字符串值（非空）

        异常：
        - RuntimeError：当参数缺失或类型不匹配时抛出
        """
        p = self.get_parameter(name)
        v = p.value
        if isinstance(v, str) and len(v) > 0:
            self.get_logger().info(f"参数 '{name}' 值为 {v}")
            return v
        if v is None or (isinstance(v, str) and len(v) == 0):
            self.get_logger().fatal(f"缺少必需参数 '{name}'（字符串）")
            raise RuntimeError(f"缺少必需参数 '{name}'（字符串）")
        self.get_logger().fatal(f"参数 '{name}' 类型不匹配，期望字符串，实际为 {type(v).__name__}")
        raise RuntimeError(f"参数 '{name}' 类型不匹配")

    def _require_int(self, name: str) -> int:
        p = self.get_parameter(name)
        v = p.value
        if isinstance(v, int):
            self.get_logger().info(f"参数 '{name}' 值为 {v}")
            return v
        if v is None:
            self.get_logger().fatal(f"缺少必需参数 '{name}'（整数）")
            raise RuntimeError(f"缺少必需参数 '{name}'（整数）")
        self.get_logger().fatal(f"参数 '{name}' 类型不匹配，期望整数，实际为 {type(v).__name__}")
        raise RuntimeError(f"参数 '{name}' 类型不匹配")

    def _require_float(self, name: str) -> float:
        p = self.get_parameter(name)
        v = p.value
        if isinstance(v, float):
            self.get_logger().info(f"参数 '{name}' 值为 {v}")
            return v
        if v is None:
            self.get_logger().fatal(f"缺少必需参数 '{name}'（浮点数）")
            raise RuntimeError(f"缺少必需参数 '{name}'（浮点数）")
        self.get_logger().fatal(f"参数 '{name}' 类型不匹配，期望浮点数，实际为 {type(v).__name__}")
        raise RuntimeError(f"参数 '{name}' 类型不匹配")

    def _on_chat(self, msg: Bool):
        """
        控制话题回调.

        用途：
        - 监听 Bool 控制开关；从 False->True 的沿触发一次识别启动。
        参数：
        - msg：Bool 消息，True 表示开始识别，False 表示关闭（等待端点）
        """
        val = bool(msg.data)
        # 如果上一帧为 False，当前帧为 True，且当前未在识别中，则启动识别
        if not self._chat_last and val and not self._recognizing:
            self._start_recognition()
        self._chat_last = val

    def _start_recognition(self):
        """
        启动识别线程.

        用途：
        - 防重入检查后创建后台线程执行识别逻辑。
        """
        if self._recognizing:
            return
        self._recognizing = True
        self._rec_thread = threading.Thread(target=self._run_recognition, daemon=True)
        self._rec_thread.start()
        self.get_logger().info('开始流式语音识别')

    def _publish_text(self, text: str):
        """
        发布识别文本到话题.

        参数：
        - text：识别出的字符串
        """
        # 调试信息
        self.get_logger().info(f'识别结果: {text}')
        self.pub_text.publish(String(data=text))

    def _run_recognition(self):
        """
        识别主流程（线程函数).

        用途：
        - 按 `use_cli` 分支调用 CLI 或 Python API。
        - 捕获异常并输出中文错误日志，最终重置识别状态。
        """
        try:
            if self.use_cli:
                cli = shutil.which('sherpa-onnx-alsa')
                if not cli:
                    raise RuntimeError('未找到 sherpa-onnx-alsa，可将 use_cli 置为 False 使用Python API')
                cmd = (
                    f"{cli} --provider={shlex.quote(self.provider)} "
                    f"--encoder={shlex.quote(self.encoder)} "
                    f"--decoder={shlex.quote(self.decoder)} "
                    f"--joiner={shlex.quote(self.joiner)} "
                    f"--tokens={shlex.quote(self.tokens)} "
                    f"{shlex.quote(self.alsa_device)}"
                )
                self._run_cli(cmd)
            else:
                self._run_python_api()
        except Exception as e:
            self.get_logger().error(f'识别过程发生错误：{e}')
        finally:
            self._recognizing = False
            self.get_logger().info('语音识别结束')

    def _run_cli(self, cmd: str):
        """
        以 CLI 方式运行 sherpa-onnx-alsa 并转发输出为识别结果.

        参数：
        - cmd：完整命令行字符串

        行为：
        - 逐行读取子进程输出，非空行即发布识别文本。
        - 当控制话题为 False 且超过静默时间，主动终止子进程。
        """
        self.get_logger().info(f'调用CLI: {cmd}')
        p = subprocess.Popen(
            cmd,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            bufsize=1,
            text=True,
        )
        last_line_ts = time.time()
        try:
            for line in iter(p.stdout.readline, ''):
                line = line.strip()
                if line:
                    last_line_ts = time.time()
                    self._publish_text(line)
                if (
                    self._chat_last is False
                    and (time.time() - last_line_ts) > self.silence_timeout_sec
                ):
                    p.terminate()
                    break
        finally:
            try:
                p.terminate()
            except Exception:
                pass

    def _run_python_api(self):
        """
        以 Python API 方式进行流式识别并使用端点检测控制结束.

        参考官方示例：
        https://k2-fsa.github.io/sherpa/onnx/pretrained_models/index.html
        """
        self.get_logger().info('使用Python API进行识别（端点检测）')

        # 使用 from_transducer 创建识别器
        # 参数映射参考 launch 文件与官方示例结构
        try:
            recognizer = sherpa_onnx.OnlineRecognizer.from_transducer(
                tokens=self.tokens,
                encoder=self.encoder,
                decoder=self.decoder,
                joiner=self.joiner,
                provider=self.provider,
                num_threads=1,
                sample_rate=16000,  # 模型采样率
                feature_dim=80,
                enable_endpoint_detection=True,
                rule1_min_trailing_silence=self.rule1_min_trailing_silence,
                rule2_min_trailing_silence=self.rule2_min_trailing_silence,
                rule3_min_utterance_length=self.rule3_min_utterance_length,
                decoding_method="greedy_search",
            )
        except Exception as e:
            self.get_logger().error(f'创建识别器失败: {e}')
            return

        stream = recognizer.create_stream()

        # 麦克风采样率 48k，模型内部重采样
        sample_rate = 48000
        samples_per_read = int(0.1 * sample_rate)  # 0.1秒

        # 标志位：是否已经发送了 START 信号
        has_sent_start = False
        # 缓存：上一次发布的内容，用于计算增量
        self.last_published_text = ""

        try:
            # 使用 sounddevice 阻塞读取模式
            with sd.InputStream(
                channels=1,
                dtype="float32",
                samplerate=sample_rate,
                device=self.target_device_idx
            ) as s:
                # 只要节点仍处于识别状态，就持续循环读取音频
                while self._recognizing:
                    # 每次阻塞读取 0.1 秒的音频数据（samples_per_read 个采样点）
                    # 返回的 samples 形状为 (samples_per_read, 1)，_ 为溢出标志（此处忽略）
                    samples, _ = s.read(samples_per_read)
                    # 将二维数组 reshape 成一维，方便后续波形输入
                    samples = samples.reshape(-1)
                    # 将原始波形喂给识别流，内部会做重采样到模型所需 16kHz
                    stream.accept_waveform(sample_rate, samples)

                    # 当识别器准备好且有待解码数据时，持续解码
                    # 每次解码会更新内部状态，可能产生新识别结果
                    while recognizer.is_ready(stream):
                        recognizer.decode_stream(stream)

                    # 检查当前是否达到端点（一句话结束）
                    is_endpoint = recognizer.is_endpoint(stream)
                    # 获取当前识别结果（包含已识别文本）
                    result = recognizer.get_result(stream)
                    
                    # 调试：计算音频最大振幅，判断是否有声音输入
                    if samples.size > 0:
                        max_amp = float(max(abs(samples.min()), abs(samples.max())))
                        if max_amp >= 1.0:
                             self.get_logger().warn(f"音频输入饱和/削波 (Max Amp: {max_amp:.2f})！请检查麦克风增益或环境噪音。")
                        elif max_amp > 0.1: # 仅在音量较大时打印，避免刷屏
                             self.get_logger().debug(f"Audio input detected, max amp: {max_amp:.2f}")

                    # 提取识别文本并去掉首尾空白
                    if isinstance(result, str):
                        text = result
                    elif hasattr(result, 'text'):
                        text = result.text
                    else:
                        text = ''
                    
                    text = text.strip()
                    
                    # 关键逻辑修改：去重处理
                    # 只有当本次识别到的文本比上一次发布的文本更长，且以其为前缀时，才发布新增部分。
                    # 或者如果这是新的开始（has_sent_start=False），则直接发布。
                    # 注意：流式ASR通常会输出不断变长的中间结果（如 "我", "我想", "我想你"）。
                    # 下游 LLM 如果只是简单叠加，就会变成 "我我想我想你"。
                    # 策略：不发布中间结果，只发布端点（整句）结果？或者发布全量但下游做去重？
                    # 鉴于用户描述 LLM 是“连续监听话题，有内容就叠加”，那么 ASR 应该只发布“增量”内容，或者只在 END 时发布完整内容。
                    # 但为了低延迟，我们应该发布增量。
                    # 可是 sherpa-onnx 的 get_result 返回的是当前整句的完整识别结果。
                    
                    # 解决方案：维护一个 last_published_text，计算增量。
                    if text:
                        # 如果是本句对话的第一个有效文本，先发送 START
                        if not has_sent_start:
                            self._publish_text("START")
                            has_sent_start = True
                            self.last_published_text = "" # 新句子开始，重置去重缓存
                        
                        # 计算增量：如果 text 是 last_published_text 的延续
                        if text.startswith(self.last_published_text):
                            new_content = text[len(self.last_published_text):]
                            if new_content:
                                self._publish_text(new_content)
                                self.last_published_text = text
                        else:
                            # 这种情况比较少见（修正了之前的识别错误），全量发布新版本或者只发布差异？
                            # 简单起见，如果发生了回溯修正，这里可能需要更复杂的逻辑。
                            # 现在的简单策略：直接发布当前完整 text，但这样可能会导致重复。
                            # 更稳妥的策略：流式ASR场景下，如果下游是叠加的，ASR最好只输出“稳定”的增量。
                            # 但 sherpa-onnx 默认输出包含非稳定部分。
                            # 为了解决用户的“重复叠加”问题，最简单的办法是：只在 END 时发布一次最终结果？
                            # 不行，那样延迟太高。
                            # 采用增量发布：
                            pass # 上面的 startswith 已经处理了正常增长的情况。
                            # 如果 text 变短了或者变了（修正），暂不处理，等待更稳定的结果。
                        
                    # 只要检测到端点（无论是否有文本），都需要处理流的重置或退出
                    if is_endpoint:
                        # 调试：如果检测到端点，强制打印结果状态
                        self.get_logger().debug(f"端点检测触发。识别文本: '{text}'")
                        
                        # 关键修改：一旦检测到端点，且之前发送过START（说明有有效语音），立即发送END
                        if has_sent_start:
                             self._publish_text("END")
                             has_sent_start = False # 重置标志，等待下一句话
                             self.last_published_text = "" # 重置去重缓存

                        if not self._chat_last:
                            # 非连续模式：结束循环
                            break
                        else:
                            # 连续识别模式：重置流，准备下一句
                            self.get_logger().debug("连续识别模式：重置识别流")
                            recognizer.reset(stream)

        except Exception as e:
            # 捕获音频流读取或识别过程中的任何异常，记录中文错误日志
            self.get_logger().error(f'音频流读取或识别异常: {e}')

        # 循环结束后的清理：如果因异常或其他原因退出循环，且处于未闭合状态，补发END
        if has_sent_start:
             self._publish_text("END")
             self.last_published_text = ""


def main(args=None):
    """程序入口：初始化 ROS 2 并运行节点."""
    rclpy.init(args=args)
    node = AudioAsrNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
