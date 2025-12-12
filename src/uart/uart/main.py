import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, UInt32
from rclpy.parameter import Parameter  # noqa: F401 (兼容旧版本，不直接使用枚举类型)
import serial
import threading
import struct
import time


START_FLAG = 0xAA
END_FLAG = 0x55


class UartNode(Node):
    """
    串口上报解析节点（uart）

    参数约定（在 uart.launch.py 中设置）：
    - port: 串口设备路径，例如 '/dev/ttyS3'
    - baud: 波特率，例如 460800
    - chat_topic: 发布 chat_gpt_enable 的话题，例如 '/uart/chat_gpt_enable'
    - timestamp_topic: 发布 timestamp 的话题，例如 '/uart/timestamp'

    数据帧格式：
    - start_flag(0xAA) + length(1B,payload长度) + payload(length B) + end_flag(0x55)
    - 当前 payload 布局：
        [0]: chat_gpt_enable (uint8, 0/1)
        [1..4]: timestamp (uint32, little-endian)
      后续扩展字段可在 _parse() 中追加解析逻辑。
    """

    def __init__(self):  # 构造函数：初始化节点、读取参数、启动读串口线程
        super().__init__(
            'uart',
            allow_undeclared_parameters=False,  # 严格模式：不允许未声明参数
            automatically_declare_parameters_from_overrides=True,  # 从 launch/CLI 自动声明参数
        )
        # 读取运行参数（以 launch 为准），源码不提供后备默认值，缺失视为错误
        self.port = self._require_str('port')  # 读取串口设备路径
        self.baud = self._require_int('baud')  # 读取串口波特率
        chat_topic = self._require_str('chat_topic')  # 读取 chat_gpt_enable 话题名
        timestamp_topic = self._require_str('timestamp_topic')  # 读取 timestamp 话题名

        # 话题发布器
        self.pub_chat = self.create_publisher(Bool, chat_topic, 10)  # 创建布尔类型发布器，用于发布 chat_gpt_enable
        self.pub_ts = self.create_publisher(UInt32, timestamp_topic, 10)  # 创建 32 位无符号整型发布器，用于发布时间戳

        self._ser = None  # 串口对象占位（未打开时为 None）
        self._buf = bytearray()  # 接收缓冲区，用于拼接与解析完整帧
        self._stop = threading.Event()  # 停止事件，用于通知读线程退出
        self._thread = threading.Thread(target=self._run, daemon=True)  # 后台读串口线程，守护模式
        self._open()  # 尝试打开串口
        self._thread.start()  # 启动读串口线程

    def _open(self):  # 打开串口：根据当前参数配置 Serial 对象
        try:  # 捕获打开过程中的异常
            self._ser = serial.Serial(  # 构造并打开串口
                self.port,  # 串口设备路径
                self.baud,  # 串口波特率
                bytesize=serial.EIGHTBITS,  # 数据位：8 位
                parity=serial.PARITY_NONE,  # 校验位：无
                stopbits=serial.STOPBITS_ONE,  # 停止位：1 位
                timeout=0,  # 读超时：非阻塞，立即返回
                write_timeout=1,  # 写超时：1 秒
            )
            self.get_logger().info(f'已打开串口 {self.port}，波特率 {self.baud}')  # 打印打开成功日志（中文）
        except Exception as e:  # 打开失败时进入异常分支
            self.get_logger().error(f'打开串口失败：{e}')  # 打印错误信息（中文）
            self._ser = None  # 标记串口未打开

    def _run(self):  # 后台线程主体：循环读取串口数据并驱动解析
        while not self._stop.is_set():  # 当未收到停止事件时持续运行
            if self._ser is None or not self._ser.is_open:  # 串口未打开或已被关闭
                self._open()  # 尝试重新打开串口
                if self._ser is None:  # 若仍失败
                    time.sleep(1.0)  # 休眠 1 秒后重试，避免频繁尝试
                    continue  # 进入下一轮循环
            try:  # 正常读取数据
                n = self._ser.in_waiting  # 查询接收缓冲区中待读的字节数
                if n:  # 如果有数据待读
                    data = self._ser.read(n)  # 读取所有可用数据
                    if data:  # 确认读取到非空数据
                        self._buf.extend(data)  # 追加到解析缓冲区
                        # 调试：打印HEX数据
                        self.get_logger().debug(f'收到数据：{[hex(b) for b in data]}')
                        self._parse()  # 触发解析流程（可能解析出多帧）
                else:  # 没有数据
                    time.sleep(0.001)  # 短暂休眠，降低 CPU 占用
            except Exception as e:  # 读取过程中发生异常
                self.get_logger().error(f'读取串口数据出错：{e}')  # 打印读取错误（中文）
                try:
                    self._ser.close()  # 尝试关闭串口，释放资源
                except Exception:
                    pass  # 忽略关闭失败
                self._ser = None  # 置空串口对象，下一轮将尝试重开

    def _parse(self):  # 从缓冲区提取完整帧并解析字段，发布对应话题
        b = self._buf  # 引用内部缓冲区（bytearray），便于切片与删除
        self.get_logger().debug(f'进入解析：缓冲区长度={len(b)}')
        while True:  # 循环尝试解析，直到没有足够数据
            if len(b) < 2:  # 至少需要起始标志与长度字节
                self.get_logger().debug(f'缓冲不足，len(b)={len(b)}，等待更多数据')
                break  # 数据不足，等待后续字节
            try:
                idx = b.index(START_FLAG)  # 搜索起始标志 0xAA 的位置
            except ValueError:  # 未找到起始标志，说明缓冲为噪声
                self.get_logger().warning('未找到起始标志0xAA，清空缓冲后等待新数据')
                self._buf.clear()  # 清空缓冲，防止无限增长
                break  # 退出解析，等待新数据
            if idx:  # 起始标志不在第 0 位，丢弃起始前的噪声字节
                self.get_logger().debug(f'丢弃起始前噪声字节数={idx}，噪声HEX={[(hex(x)) for x in b[:idx]]}')
                del b[:idx]
                if len(b) < 2:  # 丢弃后可能不足两个字节
                    self.get_logger().debug(f'丢弃噪声后缓冲不足，len(b)={len(b)}')
                    break
            length_field = b[1]  # 长度字段（设备定义）
            # 尝试两种解释：
            # 1) length_field 为整帧总长度（含 start/length/payload/end）
            total_len = length_field
            ok_total = len(b) >= total_len and b[total_len - 1] == END_FLAG
            # 2) length_field 为纯载荷长度（不含 start/length/end）
            payload_len = length_field
            total_payload = 1 + 1 + payload_len + 1
            ok_payload = len(b) >= total_payload and b[total_payload - 1] == END_FLAG

            if ok_total:
                total = total_len
                frame = b[:total]
                del b[:total]
                payload = frame[2:-1]
                self.get_logger().debug(
                    f'采用整帧长度解释：total={total}，payload_len={len(payload)}，payloadHEX={[(hex(x)) for x in payload]}'
                )
            elif ok_payload:
                total = total_payload
                frame = b[:total]
                del b[:total]
                payload = frame[2:2 + payload_len]
                self.get_logger().debug(
                    f'采用载荷长度解释：payload_len={payload_len}，total={total}，payloadHEX={[(hex(x)) for x in payload]}'
                )
            else:
                # 判断是否已有足够字节但结束标志不匹配，若是则滑动一个字节重同步
                if len(b) >= total_len and b[total_len - 1] != END_FLAG:
                    self.get_logger().warning(
                        f'候选整帧长度结束标志错误：len(b)={len(b)}，total_len={total_len}，实际={hex(b[total_len - 1])}，期望={hex(END_FLAG)}；滑动一个字节重同步'
                    )
                    del b[0]
                    continue
                if len(b) >= total_payload and b[total_payload - 1] != END_FLAG:
                    self.get_logger().warning(
                        f'候选载荷长度结束标志错误：len(b)={len(b)}，total_payload={total_payload}，实际={hex(b[total_payload - 1])}，期望={hex(END_FLAG)}；滑动一个字节重同步'
                    )
                    del b[0]
                    continue
                # 否则字节仍不足，等待后续数据
                self.get_logger().debug(
                    f'帧不完整或结束标志待确认：len(b)={len(b)}，length_field={length_field}，'
                    f'candidate_total={total_len} 或 {total_payload}'
                )
                break
            chat = None  # chat_gpt_enable 解析结果占位
            ts = None  # timestamp 解析结果占位
            if len(payload) >= 1:
                chat = bool(payload[0])
                self.get_logger().debug(f'字段解析：chat_gpt_enable={chat}')
            if len(payload) >= 5:
                ts = struct.unpack('<I', payload[1:5])[0]
            if chat is not None:  # 有效则发布 chat 话题
                self.pub_chat.publish(Bool(data=chat))
            if ts is not None:  # 有效则发布 timestamp 话题
                self.pub_ts.publish(UInt32(data=ts))
            # if chat is not None or ts is not None:  # 调试：打印解析日志
                self.get_logger().debug(f'解析上行：ChatGPT启用={chat}，时间戳={ts}，载荷长度={len(payload)}')

    def destroy_node(self):  # 节点销毁：停止读线程并释放串口资源
        self._stop.set()  # 设置停止事件，通知后台线程退出
        try:
            if self._thread.is_alive():  # 若线程仍在运行
                self._thread.join(timeout=1.0)  # 等待最多 1 秒，让其正常结束
        except Exception:
            pass  # 忽略等待过程中产生的异常
        try:
            if self._ser:  # 若串口对象存在
                self._ser.close()  # 关闭串口
        except Exception:
            pass  # 忽略关闭异常，避免影响后续销毁
        return super().destroy_node()  # 调用父类销毁逻辑

    # 工具方法：严格读取必需参数（缺失即报错退出）
    def _require_str(self, name: str) -> str:
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


def main(args=None):
    rclpy.init(args=args)
    node = UartNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
