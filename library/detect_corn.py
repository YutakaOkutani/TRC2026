# encoding: utf-8
import time

import cv2
import numpy as np
from picamera2 import Picamera2


class detector:
    def __init__(self):
        # Cone-like shape preference (used only for mid-range confidence scoring)
        self.cone_ratio = 33 / 70
        self.ratio_thresh = 0.25

        # Goal (close contact) judgment: screen should be mostly red and touching edges
        # Goal判定はやや厳しめにする（近距離で画面を大きく占有していること）
        self.reached_occupancy_thresh = 0.68
        self.reached_edge_touch_min = 3

        # Runtime state
        self.input_img = None
        self.projected_img = None
        self.binarized_img = None
        self.detected = None
        self.probability = 0.0
        self.centroids = None
        self.cone_direction = None
        self.occupancy = 0.0
        self.frame_red_occupancy = 0.0
        self.is_detected = False
        self.is_reached = False
        self.debug_method = "init"

        # Camera
        self.picam2 = None
        self.camera_width = 640
        self.camera_height = 480
        self.camera_warmup_sec = 0.8
        self.capture_retry_count = 3
        self.capture_retry_sleep = 0.2

        # ROI histogram (None = default red mode)
        self.__roi_hist = None

        # Default red HSV ranges (OpenCV H: 0-179)
        self.default_hsv_ranges = [
            (np.array([0, 70, 50], dtype=np.uint8), np.array([12, 255, 255], dtype=np.uint8)),
            (np.array([168, 70, 50], dtype=np.uint8), np.array([179, 255, 255], dtype=np.uint8)),
        ]

    def set_roi_img(self, roi):
        if roi is None:
            print("[Detector] Warning: ROI image is None. Using default color range.")
            self.__roi_hist = None
            return
        try:
            roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            hist = cv2.calcHist([roi_hsv], [0, 1], None, [180, 256], [0, 180, 0, 256])
            cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX)
            self.__roi_hist = hist
            print("[Detector] ROI Histogram set successfully.")
        except Exception as exc:
            print(f"[Detector] Error setting ROI: {exc}. Using default color.")
            self.__roi_hist = None

    def __init_camera(self):
        try:
            if self.picam2 is not None:
                try:
                    self.picam2.stop()
                except Exception:
                    pass
                try:
                    self.picam2.close()
                except Exception:
                    pass

            self.picam2 = Picamera2()
            config = self.picam2.create_preview_configuration(
                main={"size": (self.camera_width, self.camera_height), "format": "BGR888"}
            )
            self.picam2.configure(config)
            self.picam2.start()
            time.sleep(self.camera_warmup_sec)
            print("[Detector] Camera Initialized.")
            return True
        except Exception as exc:
            print(f"[Detector] Camera Init Failed: {exc}")
            self.picam2 = None
            return False

    def __get_camera_img(self):
        if self.picam2 is None and not self.__init_camera():
            return None

        for attempt in range(self.capture_retry_count):
            try:
                raw = self.picam2.capture_array()
                return cv2.blur(raw, (5, 5))
            except Exception as exc:
                print(f"[Detector] Capture Error (attempt {attempt + 1}/{self.capture_retry_count}): {exc}")
                time.sleep(self.capture_retry_sleep)

        self.__init_camera()
        return None

    def __red_mask(self, bgr_img):
        hsv = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
        masks = [cv2.inRange(hsv, lo, hi) for lo, hi in self.default_hsv_ranges]
        mask = masks[0]
        for extra in masks[1:]:
            mask = cv2.bitwise_or(mask, extra)
        return hsv, mask

    def __back_projection_mask(self, hsv_img):
        if self.__roi_hist is None:
            return None, None
        proj = cv2.calcBackProject([hsv_img], [0, 1], self.__roi_hist, [0, 180, 0, 256], 1)
        proj = cv2.GaussianBlur(proj, (9, 9), 0)
        _, bp_bin = cv2.threshold(proj, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        return proj, bp_bin

    def __postprocess_mask(self, mask):
        if mask is None:
            return None
        kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
        kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        out = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)
        out = cv2.morphologyEx(out, cv2.MORPH_OPEN, kernel_open)
        return out

    def __edge_touch_count(self, mask):
        if mask is None or mask.size == 0:
            return 0
        m = mask > 0
        count = 0
        if np.any(m[0, :]):
            count += 1
        if np.any(m[-1, :]):
            count += 1
        if np.any(m[:, 0]):
            count += 1
        if np.any(m[:, -1]):
            count += 1
        return count

    def __component_metrics(self, mask):
        if mask is None:
            return None
        mask_u8 = mask.astype(np.uint8)
        nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_u8)
        if nlabels <= 1:
            return None

        img_size = float(self.camera_width * self.camera_height)
        best = None
        for idx in range(1, nlabels):
            s = stats[idx]
            area = float(s[cv2.CC_STAT_AREA])
            if area / img_size < 0.001:
                continue
            w = max(1, int(s[cv2.CC_STAT_WIDTH]))
            h = max(1, int(s[cv2.CC_STAT_HEIGHT]))
            cx, cy = centroids[idx]
            aspect = float(w) / float(h)
            aspect_diff = abs(aspect - self.cone_ratio)
            shape_score = 1.0 - min(aspect_diff / max(self.ratio_thresh, 1e-6), 1.0)
            area_score = min((area / img_size) / 0.10, 1.0)
            center_score = 1.0 - min(abs((cx / self.camera_width) - 0.5) / 0.5, 1.0)
            score = 0.55 * area_score + 0.30 * shape_score + 0.15 * center_score
            item = {
                "label_idx": idx,
                "score": score,
                "area": area,
                "occupancy": area / img_size,
                "bbox": [
                    int(s[cv2.CC_STAT_LEFT]),
                    int(s[cv2.CC_STAT_TOP]),
                    int(s[cv2.CC_STAT_WIDTH]),
                    int(s[cv2.CC_STAT_HEIGHT]),
                ],
                "centroid": [int(cx), int(cy)],
                "shape_score": shape_score,
                "aspect": aspect,
                "labels": labels,
            }
            if best is None or item["score"] > best["score"]:
                best = item
        return best

    def __close_range_reached(self, red_mask):
        img_size = float(self.camera_width * self.camera_height)
        red_occ = float(np.count_nonzero(red_mask)) / img_size if red_mask is not None else 0.0
        edge_touch = self.__edge_touch_count(red_mask)
        reached = (red_occ >= self.reached_occupancy_thresh) and (edge_touch >= self.reached_edge_touch_min)
        return reached, red_occ, edge_touch

    def __build_candidate(self, bgr_img, variant_name):
        hsv, red_mask_raw = self.__red_mask(bgr_img)
        proj, bp_mask_raw = self.__back_projection_mask(hsv)

        red_mask = self.__postprocess_mask(red_mask_raw)
        bp_mask = self.__postprocess_mask(bp_mask_raw) if bp_mask_raw is not None else None

        # Prefer overlap when ROI histogram is available, but fall back gracefully.
        if bp_mask is not None:
            overlap = cv2.bitwise_and(red_mask, bp_mask)
            union = cv2.bitwise_or(red_mask, bp_mask)
            overlap = self.__postprocess_mask(overlap)
            union = self.__postprocess_mask(union)

            candidates = [
                ("hybrid_overlap", overlap),
                ("hybrid_union", union),
                ("backproj", bp_mask),
                ("hue", red_mask),
            ]
        else:
            candidates = [("hue", red_mask)]

        best_mode = None
        best_component = None
        best_mask = None
        best_mode_score = -1.0
        for mode_name, mask in candidates:
            comp = self.__component_metrics(mask)
            comp_score = comp["score"] if comp is not None else 0.0
            # Slight preference for hue-based masks in close range because shape can collapse.
            if mode_name.startswith("hue"):
                comp_score += 0.02
            if comp_score > best_mode_score:
                best_mode_score = comp_score
                best_mode = mode_name
                best_component = comp
                best_mask = mask

        reached, frame_red_occupancy, edge_touch = self.__close_range_reached(red_mask)

        prob = 0.0
        centroid = None
        cone_dir = None
        occupancy = 0.0
        bbox = None
        is_detected = False

        if best_component is not None:
            bbox = best_component["bbox"]
            centroid = best_component["centroid"]
            occupancy = best_component["occupancy"]
            prob = float(max(0.0, min(1.0, best_component["score"])))
            cone_dir = float(best_component["centroid"][0]) / float(self.camera_width)
            is_detected = True

        # If the cone is too close, the component shape often breaks. Promote close-range evidence.
        if reached:
            is_detected = True
            prob = 1.0
            if centroid is None and red_mask is not None and np.count_nonzero(red_mask) > 0:
                m = cv2.moments(red_mask)
                if m["m00"] > 0:
                    cx = int(m["m10"] / m["m00"])
                    cy = int(m["m01"] / m["m00"])
                    centroid = [cx, cy]
                    cone_dir = float(cx) / float(self.camera_width)

        if cone_dir is None:
            cone_dir = 0.5

        # Candidate quality to resolve RGB/BGR ambiguity across camera setups.
        overall_score = (
            (2.0 if reached else 0.0)
            + prob
            + min(frame_red_occupancy / 0.6, 1.0)
            + (0.1 if self.__roi_hist is not None and best_mode and "hybrid" in best_mode else 0.0)
        )

        return {
            "variant_name": variant_name,
            "bgr_img": bgr_img,
            "projected_img": proj if proj is not None else red_mask_raw,
            "binarized_img": best_mask if best_mask is not None else red_mask,
            "bbox": bbox,
            "centroid": centroid,
            "cone_direction": cone_dir,
            "occupancy": occupancy,
            "frame_red_occupancy": frame_red_occupancy,
            "is_detected": is_detected,
            "is_reached": reached,
            "probability": prob,
            "debug_method": f"{variant_name}:{best_mode or 'none'}",
            "overall_score": overall_score,
            "edge_touch_count": edge_touch,
        }

    def detect_cone(self):
        self.is_detected = False
        self.is_reached = False
        self.probability = 0.0
        self.cone_direction = None
        self.occupancy = 0.0
        self.frame_red_occupancy = 0.0
        self.centroids = None
        self.projected_img = None
        self.binarized_img = None
        self.debug_method = "reset"

        raw_img = self.__get_camera_img()
        if raw_img is None:
            return False

        # Try both channel assumptions. Some deployments behave like RGB even with BGR888 config.
        variant_bgr = raw_img
        variant_swap_rb = cv2.cvtColor(raw_img, cv2.COLOR_RGB2BGR)

        cand1 = self.__build_candidate(variant_bgr, "as_is")
        cand2 = self.__build_candidate(variant_swap_rb, "swap_rb")
        best = cand1 if cand1["overall_score"] >= cand2["overall_score"] else cand2

        self.input_img = best["bgr_img"]
        self.projected_img = best["projected_img"]
        self.binarized_img = best["binarized_img"]
        self.detected = best["bbox"]
        self.probability = float(best["probability"])
        self.centroids = np.array(best["centroid"], dtype=float) if best["centroid"] is not None else None
        self.cone_direction = float(best["cone_direction"])
        self.occupancy = float(best["occupancy"])
        self.frame_red_occupancy = float(best["frame_red_occupancy"])
        self.is_detected = bool(best["is_detected"])
        self.is_reached = bool(best["is_reached"])
        self.debug_method = best["debug_method"]

        if self.is_reached:
            try:
                self.picam2.capture_file("./log/capture_reached.png")
            except Exception:
                pass

        return True
