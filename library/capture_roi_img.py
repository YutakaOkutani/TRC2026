import cv2
import numpy as np
from picamera2 import Picamera2
import time
import os

print("Capture ROI Start")
os.makedirs("./log", exist_ok=True) # フォルダ作成

try:
    picam2 = Picamera2()
    # 検出と同じ解像度で撮るのがベスト
    config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "BGR888"})
    picam2.configure(config)
    picam2.start()

    # ホワイトバランス安定待ち
    time.sleep(2)

    print("Capturing...")
    # 保存名は main.py で探す名前と一致させる
    picam2.capture_file("./log/captured_roi_img.png")
    print("Saved to ./log/captured_roi_img.png")
    
    picam2.stop()
    picam2.close()

except Exception as e:
    print(f"Error: {e}")