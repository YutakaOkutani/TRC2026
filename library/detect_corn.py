# encoding : utf-8
import cv2
import numpy as np
import time
from picamera2 import Picamera2

class detector:
    def __init__(self):
        # パラメータ設定
        self.cone_ratio = 33 / 70  # コーンの縦横比
        self.ratio_thresh = 0.1    # 許容誤差
        self.reached_occupancy_thresh = 0.8 # 到達判定の面積閾値

        # 状態変数
        self.input_img = None
        self.projected_img = None
        self.binarized_img = None
        self.detected = None
        self.probability = 0.0     # 初期値0.0
        self.centroids = None
        self.cone_direction = None # None or 0.0-1.0
        self.occupancy = 0.0
        self.is_detected = False
        self.is_reached = False
        
        # カメラ関連
        self.picam2 = None
        self.camera_width = 640
        self.camera_height = 480
        
        # ROIヒストグラム (Noneならデフォルト色を使用)
        self.__roi_hist = None
        
        # デフォルトの赤色範囲 (HSV) - ファイル読み込み失敗時の命綱
        # Hが0/180をまたぐため2レンジで扱う (OpenCVのHは0-179)
        self.default_hsv_ranges = [
            (np.array([0, 100, 100]), np.array([10, 255, 255])),
            (np.array([170, 100, 100]), np.array([179, 255, 255])),
        ]

    def set_roi_img(self, roi):
        """
        目標とする色の画像をセットし、ヒストグラムを作成する。
        画像がNoneの場合は無視する（デフォルト色モードになる）。
        """
        if roi is None:
            print("[Detector] Warning: ROI image is None. Using default color range.")
            self.__roi_hist = None
            return

        try:
            self.__roi = roi
            self.__roi_hsv = cv2.cvtColor(self.__roi, cv2.COLOR_BGR2HSV)
            # ヒストグラム算出 (HとSだけを見る)
            self.__roi_hist = cv2.calcHist(
                [self.__roi_hsv], [0, 1], None, [180, 256], [0, 180, 0, 256]
            )
            cv2.normalize(self.__roi_hist, self.__roi_hist, 0, 255, cv2.NORM_MINMAX)
            print("[Detector] ROI Histogram set successfully.")
        except Exception as e:
            print(f"[Detector] Error setting ROI: {e}. Using default color.")
            self.__roi_hist = None

    def __init_camera(self):
        """カメラの初期化（失敗したらFalseを返す）"""
        try:
            if self.picam2 is not None:
                self.picam2.stop()
                self.picam2.close()
            
            self.picam2 = Picamera2()
            # 処理負荷軽減のため解像度を固定 (640x480)
            config = self.picam2.create_preview_configuration(
                main={"size": (self.camera_width, self.camera_height), "format": "BGR888"}
            )
            self.picam2.configure(config)
            self.picam2.start()
            print("[Detector] Camera Initialized.")
            return True
        except Exception as e:
            print(f"[Detector] Camera Init Failed: {e}")
            self.picam2 = None
            return False

    def __get_camera_img(self):
        """画像取得（失敗時は再接続を試みる）"""
        try:
            if self.picam2 is None:
                if not self.__init_camera():
                    return False
            
            # 画像取得
            raw_img = self.picam2.capture_array()
            # ノイズ除去
            self.input_img = cv2.blur(raw_img, (5, 5)) 
            return True
            
        except Exception as e:
            print(f"[Detector] Capture Error: {e}")
            # エラーが出たらカメラをリセットしてみる
            self.__init_camera()
            return False

    def detect_cone(self):
        """メイン検出ループ"""
        # 値のリセット
        self.is_detected = False
        self.cone_direction = None
        self.probability = 0.0
        
        # 画像取得
        if not self.__get_camera_img():
            return # カメラダメなら終了

        # 検出処理
        try:
            if self.__roi_hist is not None:
                self.__back_projection() # ヒストグラムがある場合（ファイル読み込み成功時）
            else:
                self.__simple_threshold() # ヒストグラムがない場合（デフォルト色）

            self.__binarization_post_process() # 共通の後処理
            self.__find_cone_centroid()        # 重心探索
            
        except Exception as e:
            print(f"[Detector] Process Error: {e}")
            self.is_detected = False

    def __back_projection(self):
        """逆投影法: ヒストグラムに近い色を抽出"""
        img_hsv = cv2.cvtColor(self.input_img, cv2.COLOR_BGR2HSV)
        self.projected_img = cv2.calcBackProject(
            [img_hsv], [0, 1], self.__roi_hist, [0, 180, 0, 256], 1
        )

    def __simple_threshold(self):
        """単純二値化: デフォルトのHSV範囲で抽出（フォールバック用）"""
        img_hsv = cv2.cvtColor(self.input_img, cv2.COLOR_BGR2HSV)
        masks = [cv2.inRange(img_hsv, lower, upper) for lower, upper in self.default_hsv_ranges]
        mask = masks[0]
        for extra in masks[1:]:
            mask = cv2.bitwise_or(mask, extra)
        self.projected_img = mask # back_projectionの結果と同じ形式（グレースケール）にする

    def __binarization_post_process(self):
        """二値化画像へのモルフォロジー処理"""
        # BackProjectionの場合は確率画像なので二値化が必要
        if self.__roi_hist is not None:
            ret, th = cv2.threshold(
                self.projected_img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
            )
        else:
            # simple_thresholdの場合は既に二値画像(mask)だが、一応
            th = self.projected_img

        # ノイズ除去（クロージングで穴埋め）
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15)) # 少し小さくして高速化
        self.binarized_img = cv2.morphologyEx(th, cv2.MORPH_CLOSE, kernel)

    def __find_cone_centroid(self):
        """ラベリングしてコーンらしい物体を探す"""
        if self.binarized_img is None: return

        # 画像全体の面積
        imgSize = self.camera_width * self.camera_height
        
        # ラベリング
        nlabels, labels_img, stats, centroids = cv2.connectedComponentsWithStats(
            self.binarized_img.astype(np.uint8)
        )

        if nlabels <= 1: # 背景のみ
            self.is_detected = False
            return

        # 背景(index 0)を除外
        stats = stats[1:]
        centroids = centroids[1:]

        # 面積順などでフィルタリングしたいが、まずは「最大面積」かつ「縦長」なものを探す
        
        # 評価値計算: 面積比率
        occupacies = stats[:, cv2.CC_STAT_AREA] / imgSize
        
        # 評価値計算: アスペクト比 (width/height) が cone_ratio に近いか
        # 0に近いほどコーンらしい
        aspects = stats[:, cv2.CC_STAT_WIDTH] / stats[:, cv2.CC_STAT_HEIGHT]
        diff_ratios = np.abs(aspects - self.cone_ratio)

        # 候補選定
        # 条件1: 画面の0.1%以上を占めていること (ゴミ除去)
        valid_indices = np.where(occupacies > 0.001)[0]
        
        if len(valid_indices) == 0:
            self.is_detected = False
            return

        # 有効な候補の中で、最も面積が大きいものを選ぶ（またはアスペクト比が良いもの）
        # ここでは「一番でかい塊」をコーンとみなす（単純化）
        best_idx = valid_indices[np.argmax(occupacies[valid_indices])]
        
        # 結果格納
        self.is_detected = True
        self.occupancy = occupacies[best_idx]
        self.probability = 1.0 - min(diff_ratios[best_idx], 1.0) # アスペクト比が近いほど高スコア
        self.centroids = centroids[best_idx]
        
        # 方向 (0.0:左端, 1.0:右端, 0.5:中央)
        self.cone_direction = self.centroids[0] / self.camera_width

        #   reached_occupancy_thresh を超えたら「密着（到達）」とみなす
        if  self.occupancy > self.reached_occupancy_thresh:
            self.is_reached = True
        # 到達時の記念撮影（エラーで落ちないようにtry）
            try:
                self.picam2.capture_file("./log/capture_reached.png")
            except:
                pass
        else:
            self.is_reached = False
