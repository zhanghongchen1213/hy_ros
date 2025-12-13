import logging
import asyncio
from aiohttp import web
import threading

class HTTPSServer:
    def __init__(self, loop, host='0.0.0.0', port=8080):
        self.loop = loop
        self.host = host
        self.port = port
        self.app = web.Application()
        self.app.router.add_get('/audio/stream', self.handle_stream)
        self.runner = None
        self.site = None
        # Queue 必须在 loop 中初始化，或者在使用前初始化
        self.audio_queue = None
        self.logger = logging.getLogger('HTTPSServer')
        self.running = False

    async def start(self):
        """启动 HTTPS 服务器"""
        # 在 loop 线程中初始化 Queue
        if self.audio_queue is None:
            self.audio_queue = asyncio.Queue()
            
        self.runner = web.AppRunner(self.app)
        await self.runner.setup()
        self.site = web.TCPSite(self.runner, self.host, self.port)
        await self.site.start()
        self.running = True
        self.logger.info(f"HTTPS 服务器已启动: http://{self.host}:{self.port}/audio/stream")

    async def stop(self):
        """停止服务器"""
        self.running = False
        if self.site:
            await self.site.stop()
        if self.runner:
            await self.runner.cleanup()
        self.logger.info("HTTPS 服务器已停止")

    def push_audio(self, audio_data: bytes):
        """
        将音频数据推送到队列中。
        此方法应该是线程安全的，供外部调用（如ROS回调）。
        """
        def _put_data():
            if self.audio_queue is not None:
                self.audio_queue.put_nowait(audio_data)
        
        try:
            # 使用 call_soon_threadsafe 确保在事件循环中操作队列
            if self.loop.is_running():
                self.loop.call_soon_threadsafe(_put_data)
        except RuntimeError:
            pass

    async def handle_stream(self, request):
        """
        处理音频流请求。
        客户端连接后，服务器将持续发送音频数据。
        """
        self.logger.info("客户端连接到音频流")
        
        if self.audio_queue is None:
             self.audio_queue = asyncio.Queue()

        response = web.StreamResponse(
            status=200,
            reason='OK',
            headers={
                'Content-Type': 'audio/mpeg', # 或者 audio/wav, audio/pcm
                'Cache-Control': 'no-cache',
                'Connection': 'keep-alive',
            }
        )
        await response.prepare(request)

        try:
            while self.running:
                # 等待音频数据
                data = await self.audio_queue.get()
                if data is None: # 结束信号
                    break
                
                await response.write(data)
                self.audio_queue.task_done()
        except Exception as e:
            self.logger.error(f"流连接关闭: {e}")
        finally:
            self.logger.info("客户端已断开")
        
        return response
