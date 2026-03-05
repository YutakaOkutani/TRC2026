import numpy as np
import time
import os

import cv2
from picamera2 import Picamera2


print("Capture ROI Start")
roi_dir = os.path.expanduser("~/library/log")
roi_path = os.path.join(roi_dir, "captured_roi_img.png")
roi_archive_path = os.path.join(roi_dir, f"captured_roi_img_{time.strftime('%Y%m%d_%H%M%S')}.png")
os.makedirs(roi_dir, exist_ok=True) # フォルダ作成

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
    picam2.capture_file(roi_path)
    picam2.capture_file(roi_archive_path)
    print(f"Saved to {roi_path}")
    print(f"Archived to {roi_archive_path}")
    
    picam2.stop()
    picam2.close()

except Exception as e:
    print(f"Error: {e}")
