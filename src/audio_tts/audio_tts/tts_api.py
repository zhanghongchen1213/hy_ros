import logging
import json
import time
import asyncio
import aiohttp
# import websockets
import base64

class BaiduTTSClient:
    def __init__(self, api_key, secret_key, voice_id=None):
        self.api_key = api_key
        self.secret_key = secret_key
        self.voice_id = voice_id
        self.access_token = None
        self.token_expires_at = 0
        self.logger = logging.getLogger('BaiduTTSClient')
        # 百度大模型声音复刻流式合成地址
        self.wss_url = "wss://aip.baidubce.com/ws/2.0/speech/publiccloudspeech/v1/voice/clone/tts"

    async def _refresh_token(self):
        """获取或刷新 Access Token"""
        if self.access_token and time.time() < self.token_expires_at:
            return

        url = "https://aip.baidubce.com/oauth/2.0/token"
        params = {
            "grant_type": "client_credentials",
            "client_id": self.api_key,
            "client_secret": self.secret_key
        }
        
        async with aiohttp.ClientSession() as session:
            async with session.post(url, params=params) as resp:
                data = await resp.json()
                if "access_token" in data:
                    self.access_token = data["access_token"]
                    self.token_expires_at = time.time() + data.get("expires_in", 2592000) - 60
                    self.logger.info("Access Token 已刷新")
                else:
                    self.logger.error(f"获取 Access Token 失败: {data}")
                    raise Exception("百度鉴权失败")

    async def synthesize(self, text, on_audio_chunk, spd=5, pit=5, vol=5, aue=3, lang="zh", dialect=""):
        """
        调用百度流式 TTS (复刻音色版)
        :param text: 要合成的文本
        :param on_audio_chunk: 回调函数，接收 bytes 音频数据
        :param spd: 语速 0-15
        :param pit: 音调 0-15
        :param vol: 音量 0-15
        :param aue: 音频格式 3=mp3, 6=wav
        :param lang: 语言 zh=中文
        :param dialect: 方言 (wuu-CN-shanghai, zh-CN-henan, zh-CN-sichuan, etc.)
        """
        if not text:
            return

        await self._refresh_token()
     # 拼接 URL 参数
        url = f"{self.wss_url}?access_token={self.access_token}&voice_id={self.voice_id}"
        
        try:
            # 解决 Python 3.10+ loop 参数移除问题
            # 改用 aiohttp 的 WebSocket 客户端，避免 websockets 库的老版本兼容性问题
            async with aiohttp.ClientSession() as session:
                async with session.ws_connect(url) as websocket:
                    # 1. 发送开始参数
                    start_payload = {
                        "type": "system.start",
                        "payload": {
                            "spd": spd,
                            "pit": pit,
                            "vol": vol,
                            "aue": aue,
                            "lang": lang,
                        },
                    }
                    
                    # 如果指定了方言，加入 payload
                    if dialect:
                        start_payload["payload"]["dialect"] = dialect
                    
                    await websocket.send_json(start_payload)
                    
                    # 2. 发送文本
                    text_payload = {
                        "type": "text",
                        "payload": {"text": text},
                    }
                    await websocket.send_json(text_payload)
                    self.logger.info(f"发送 TTS 请求文本: {text[:10]}...")

                    # 3. 发送结束信号
                    finish_payload = {"type": "system.finish"}
                    await websocket.send_json(finish_payload)

                    # 4. 接收循环
                    async for msg in websocket:
                        if msg.type == aiohttp.WSMsgType.TEXT:
                            msg_json = json.loads(msg.data)
                            # 检查系统消息
                            if msg_json.get("type") == "system.finish":
                                self.logger.info("TTS 合成结束")
                                break
                            if msg_json.get("type") == "system.error":
                                self.logger.error(f"TTS 错误: {msg_json}")
                                
                        elif msg.type == aiohttp.WSMsgType.BINARY:
                            # 二进制音频流
                            if on_audio_chunk:
                                on_audio_chunk(msg.data)
                        elif msg.type == aiohttp.WSMsgType.ERROR:
                            self.logger.error(f"WebSocket 连接异常关闭: {websocket.exception()}")
                            break
                            
        except Exception as e:
            self.logger.error(f"TTS WebSocket 异常: {e}")


    async def create_voice(self, voice_name, audio_base64, voice_desc=""):
        """
        创建百度大模型声音复刻音色
        :param voice_name: 音色名称
        :param audio_base64: Base64 编码的音频内容
        :param voice_desc: 音色描述
        :return: 成功返回 voice_id，失败抛出异常
        """
        await self._refresh_token()
        
        url = "https://aip.baidubce.com/rest/2.0/speech/publiccloudspeech/v1/voice/clone/create"
        params = {"access_token": self.access_token}
        
        payload = {
            "voice_name": voice_name,
            "voice_desc": voice_desc,
            "audio_file": audio_base64,
            # "text_id": "" # 可选
        }
        
        headers = {'Content-Type': 'application/json'}
        
        async with aiohttp.ClientSession() as session:
            async with session.post(url, params=params, json=payload, headers=headers) as resp:
                data = await resp.json()
                if resp.status == 200 and data.get("status") == 0:
                    self.logger.info(f"音色创建成功: {data}")
                    return data.get("data", {}).get("voice_id")
                else:
                    self.logger.error(f"音色创建失败: {data}")
                    raise Exception(f"Create Voice Failed: {data}")

if __name__ == "__main__":
    async def test_main():
        import os
        
        # 配置日志输出到控制台
        logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        
        # ==========================================
        # 请在此处直接修改参数进行测试
        # ==========================================
        api_key = ""      # 填入百度 API Key
        secret_key = ""   # 填入百度 Secret Key
        voice_id = ""     # 填入 Voice ID
        text = "我不是 Butter Robot，我是 Butter Robot 2.0，我是一个基于大模型的语音合成系统。"
        output_file = "test_output.mp3"
        
        # 创建音色参数
        voice_name = "黄油机器人"
        voice_desc = "测试复刻音色"
        base64_file_path = "/mnt/nfs/hy_ros/src/audio_tts/audio_tts/base64.txt"
        
        # 设置模式：True 为创建音色，False 为测试 TTS 合成
        run_create_voice = True
        
        # ==========================================
        
        if not api_key or not secret_key:
            print("错误：请在代码中填入有效的 api_key 和 secret_key")
            return

        client = BaiduTTSClient(api_key, secret_key, voice_id)
        
        if run_create_voice:
            # ... 创建音色逻辑保持不变 ...
            print(f"正在读取 Base64 音频文件: {base64_file_path}")
            try:
                with open(base64_file_path, "r") as f:
                    audio_base64 = f.read().strip()
                
                print("正在请求创建音色...")
                voice_id = await client.create_voice(voice_name, audio_base64, voice_desc)
                print(f"\n[成功] 音色创建成功！Voice ID: {voice_id}")
                print(f"请保存此 Voice ID 用于后续 TTS 合成。")
                
            except Exception as e:
                print(f"\n[失败] 创建音色发生错误: {e}")
        else:
             # 测试 TTS 合成
             if not voice_id:
                 print("错误：TTS 测试需要提供 voice_id")
                 return
                 
             print(f"\n开始合成文本: {text}")
             print(f"目标输出文件: {output_file}")
             print("正在连接百度云 API...")
             
             try:
                 with open(output_file, "wb") as f:
                     def save_audio(chunk):
                         f.write(chunk)
                         print(".", end="", flush=True)
                         
                     await client.synthesize(text, save_audio, dialect="zh-CN-henan")
                     
                 print(f"\n\n合成完成！音频已保存至: {os.path.abspath(output_file)}")
             except Exception as e:
                 print(f"\n合成失败: {e}")

    try:
        asyncio.run(test_main())
    except KeyboardInterrupt:
        pass
