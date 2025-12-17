#!/usr/bin/env python3
import cv2
import os
import sys
import tty
import termios
import re

# 配置保存路径
SAVE_DIR = "/mnt/nfs/hy_ros/source/butter_img/"

def get_next_index(directory):
    """获取下一个可用的文件序号"""
    if not os.path.exists(directory):
        os.makedirs(directory)
        return 0
    
    files = os.listdir(directory)
    max_idx = -1
    
    # 匹配纯数字命名的 .jpg 文件
    pattern = re.compile(r'^(\d+)\.jpg$')
    
    for f in files:
        match = pattern.match(f)
        if match:
            idx = int(match.group(1))
            if idx > max_idx:
                max_idx = idx
                
    return max_idx + 1

def getch():
    """读取单个字符输入，无需回车"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main():
    # 1. 初始化摄像头
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("错误：无法打开摄像头！请检查设备连接。")
        return

    # 设置分辨率 (可选)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    # 确保目录存在
    if not os.path.exists(SAVE_DIR):
        os.makedirs(SAVE_DIR)
        print(f"已创建目录: {SAVE_DIR}")

    print("="*40)
    print(f"摄像头拍照工具已启动")
    print(f"图片保存路径: {SAVE_DIR}")
    print("操作说明:")
    print("  [任意键] : 拍照并保存")
    print("  [q / ESC]: 退出程序")
    print("="*40)

    try:
        while True:
            # 2. 等待键盘输入
            key = getch()
            
            # 退出条件 (q 或 ESC)
            if key == 'q' or ord(key) == 27:
                print("\n程序已退出。")
                break
            
            # 3. 拍照
            # 清空缓冲区：连续读取几帧以获取最新图像
            for _ in range(5):
                cap.read()
            
            ret, frame = cap.read()
            if not ret:
                print("\n错误：无法获取图像帧！")
                continue
            
            # 旋转图像 180 度 (0: x轴翻转, 1: y轴翻转, -1: 两个轴都翻转)
            frame = cv2.flip(frame, -1)
                
            # 4. 获取序号并保存
            idx = get_next_index(SAVE_DIR)
            filename = f"{idx}.jpg"
            filepath = os.path.join(SAVE_DIR, filename)
            
            cv2.imwrite(filepath, frame)
            print(f"\r已保存: {filename} (序号: {idx})", end="")
            
    except KeyboardInterrupt:
        print("\n程序已中断。")
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
