import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16, UInt8
from geometry_msgs.msg import Twist
import serial
import threading
import struct
import time
import ctypes
import queue


# ==========================================
# 数据包结构定义 (与 C 代码保持一致)
# ==========================================

class UartUplinkPacket(ctypes.LittleEndianStructure):
    """
    @brief 上行数据包结构（设备上报给上位机）
    """
    _pack_ = 1
    _fields_ = [
        ("start_flag", ctypes.c_uint8),      # 0xAA
        # 补充数据包
        ("chat_gpt_count", ctypes.c_uint16),
        
        
        
        
        
        ("timestamp", ctypes.c_uint32),
        ("end_flag", ctypes.c_uint8),        # 0x55
    ]
    
    START_FLAG = 0xAA
    END_FLAG = 0x55


class UartDownlinkPacket(ctypes.LittleEndianStructure):
    """
    @brief 下行数据包结构（上位机下发给设备）
    用于下发状态标志位
    """
    _pack_ = 1
    _fields_ = [
        ("start_flag", ctypes.c_uint8),      # 0xAA
        # 补充数据包
        ("audio_stream_flag", ctypes.c_uint8), # 音频状态: 0=无, 1=ASR结束, 2=TTS结束, 3=HTTPS下发完成
        
        
        
        
        ("timestamp", ctypes.c_uint32),        # 时间戳 (ms)
        ("end_flag", ctypes.c_uint8),        # 0x66
    ]
    
    START_FLAG = 0xAA
    END_FLAG = 0x66


class UartNode(Node):
    """
    串口通讯节点（UART）
    事件驱动模式：收到 audio_stream_flag 话题更新时立即下发串口指令。
    """

    def __init__(self):
        super().__init__(
            'uart',
            allow_undeclared_parameters=False,
            automatically_declare_parameters_from_overrides=True,
        )
        
        # 1. 参数读取
        self.port = self._require_str('port')
        self.baud = self._require_int('baud')
        self.chat_topic = self._require_str('pub_chat_topic')
        self.audio_status_topic = self._require_str('sub_audio_status_topic')

        # 2. 通信资源初始化
        self._ser = None
        self._serial_lock = threading.Lock()
        self._buf = bytearray()
        self._stop = threading.Event()
        self.send_queue = queue.Queue()
        
        # 3. ROS 接口
        # [Pub] 上行
        self.pub_chat = self.create_publisher(UInt16, self.chat_topic, 10)
        
        # [Sub] 下行
        self.sub_audio_status = self.create_subscription(
            UInt8, self.audio_status_topic, self._on_audio_status, 10)
        
        # 4. 启动线程
        self._open()
        self._read_thread = threading.Thread(target=self._run_read, daemon=True)
        self._read_thread.start()
        self._write_thread = threading.Thread(target=self._run_write, daemon=True)
        self._write_thread.start()
        
        self.get_logger().info(f'UART 节点启动。事件驱动模式。端口: {self.port}')

    def _on_audio_status(self, msg: UInt8):
        """
        ROS 回调：收到音频状态更新，立即组装下行包发送。
        """
        flag = msg.data
        # self.get_logger().info(f"收到状态更新: {flag}，准备下发...")
        
        packet = UartDownlinkPacket()
        packet.start_flag = UartDownlinkPacket.START_FLAG
        packet.audio_stream_flag = flag
        packet.timestamp = int(time.time() * 1000) & 0xFFFFFFFF
        packet.end_flag = UartDownlinkPacket.END_FLAG
        
        self.send_queue.put(packet)

    def _open(self):
        with self._serial_lock:
            try:
                self._ser = serial.Serial(
                    self.port,
                    self.baud,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.01, # 读超时
                    write_timeout=1,
                )
                self.get_logger().info(f'已打开串口 {self.port}，波特率 {self.baud}')
            except Exception as e:
                self.get_logger().error(f'打开串口失败：{e}')
                self._ser = None

    def _run_write(self):
        """写线程：从队列获取数据包并写入串口"""
        while not self._stop.is_set():
            try:
                # 阻塞等待，超时方便响应停止信号
                packet = self.send_queue.get(timeout=0.5)
                
                if self._ser and self._ser.is_open:
                    data = bytes(packet)
                    with self._serial_lock:
                        self._ser.write(data)
                    self.get_logger().debug(f"发送下行数据: {[hex(b) for b in data]}")
                else:
                    self.get_logger().warning("串口未打开，丢弃发送数据")
                
                self.send_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"写串口异常: {e}")

    def _run_read(self):
        """读线程：循环读取并解析"""
        while not self._stop.is_set():
            if self._ser is None or not self._ser.is_open:
                time.sleep(1.0)
                self._open()
                continue
                
            try:
                data = b''
                if self._ser.in_waiting:
                    with self._serial_lock:
                        data = self._ser.read(self._ser.in_waiting)
                
                if data:
                    self._buf.extend(data)
                    self._parse()
                else:
                    time.sleep(0.005)
            except Exception as e:
                self.get_logger().error(f'读串口异常: {e}')
                time.sleep(1.0)

    def _parse(self):
        """解析上行数据包 (UplinkPacket)"""
        b = self._buf
        packet_size = ctypes.sizeof(UartUplinkPacket)
        
        while len(b) >= packet_size:
            # 1. 寻找起始标志
            try:
                idx = b.index(UartUplinkPacket.START_FLAG)
            except ValueError:
                b.clear()
                break
                
            if idx > 0:
                del b[:idx] # 丢弃头部噪声
                continue # 重新检查长度
            
            # 此时 b[0] == START_FLAG
            if len(b) < packet_size:
                break # 数据不够完整包
            
            # 2. 检查结束标志
            if b[packet_size - 1] != UartUplinkPacket.END_FLAG:
                del b[0]
                continue
                
            # 3. 解析数据
            try:
                # 拷贝数据到结构体
                packet_bytes = bytes(b[:packet_size])
                packet = UartUplinkPacket.from_buffer_copy(packet_bytes)
                
                # 发布话题
                self.pub_chat.publish(UInt16(data=packet.chat_gpt_count))
                
                self.get_logger().debug(
                    f"解析上行: Count={packet.chat_gpt_count}, TS={packet.timestamp}"
                )
                
                # 4. 移除已解析的数据
                del b[:packet_size]
                
            except Exception as e:
                self.get_logger().error(f"解析包内容异常: {e}")
                del b[0] # 防死循环

    def destroy_node(self):
        self._stop.set()
        if self._ser:
            self._ser.close()
        return super().destroy_node()

    def _require_str(self, name: str) -> str:
        p = self.get_parameter(name)
        v = p.value
        if isinstance(v, str) and len(v) > 0:
            return v
        raise RuntimeError(f"缺少必需字符串参数: {name}")

    def _require_int(self, name: str) -> int:
        p = self.get_parameter(name)
        v = p.value
        if isinstance(v, int):
            return v
        raise RuntimeError(f"缺少必需整数参数: {name}")


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
