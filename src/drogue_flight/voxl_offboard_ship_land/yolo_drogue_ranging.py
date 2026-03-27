# -*- coding: utf-8 -*-
"""
Created on Wed Jan 21 11:53:52 2026

@author: rllow
"""

import os
import time
import threading
import cv2
import numpy as np
from ultralytics import YOLO

# === UDP ===
import socket
import json

# =========================
# RTSP + MODEL SETTINGS
# =========================
RTSP_URL = "rtsp://127.0.0.1:8900/live"
MODEL_PATH = "../models/best.pt"

IMGSZ = 416
CONF = 0.25
IOU = 0.45
DEVICE = "cpu"  # set to "0" if GPU works

# Your stream/frame size (used by ranging center math)
FRAME_W = 1024
FRAME_H = 768

# Global PC offset (adjust if needed)
PC = np.array([0.0, 0.0, 0.0], dtype=float)

# IMPORTANT: set these to match your YOLO class IDs
CLASS_COUPLER = 0
CLASS_DROGUE = 1

# Optional save/preview
SHOW = False

# =========================
# UDP SETTINGS
# =========================
UDP_IP = "127.0.0.1"     # if ROS2 node is on same VOXL2, keep localhost
UDP_PORT = 5005
UDP_SEND_HZ = 30         # cap send rate (prevents spamming if inference spikes)
UDP_SEND_ONLY_IF_VALID = False  # set True if you only want packets when detections exist

# =========================
# LATEST-FRAME BUFFER
# =========================
LATEST = {"frame": None, "ts": 0.0, "seq": 0}
LOCK = threading.Lock()
STOP = False


def open_capture_rtsp(url: str) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(url)
    if cap.isOpened():
        return cap

    gst = (
        f'rtspsrc location={url} latency=0 drop-on-late=true ! '
        'rtph264depay ! h264parse ! avdec_h264 ! '
        'videoconvert ! appsink drop=true sync=false max-buffers=1'
    )
    return cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)


def reader_thread(cap: cv2.VideoCapture):
    global STOP

    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass

    while not STOP:
        ok, frame = cap.read()
        if not ok or frame is None:
            time.sleep(0.005)
            continue

        with LOCK:
            LATEST["frame"] = frame
            LATEST["ts"] = time.time()
            LATEST["seq"] += 1


# =========================
# RANGING
# =========================
def RangingFunction(x1c, y1c, x2c, y2c, n, frame_w=FRAME_W, frame_h=FRAME_H):
    """
    Returns [OffsetDistancex, OffsetDistancey, WeightedDistance, FinalDistance]
    """
    FocalLength = 721.7273
    CouplerWidths = {1: 10.8, 2: 26.5}  # n=1 coupler, n=2 drogue
    CouplerWidth = CouplerWidths.get(n, 10.8)

    xd = abs(x2c - x1c)
    yd = abs(y2c - y1c)
    if xd <= 1 or yd <= 1:
        return [0.0, 0.0, 0.0, 0.0]

    droguex = x1c + xd / 2.0
    droguey = y1c + yd / 2.0

    vidcenterx = frame_w / 2.0
    vidcentery = frame_h / 2.0

    StraightDistancex = (FocalLength / xd) * CouplerWidth
    StraightDistancey = (FocalLength / yd) * CouplerWidth

    CenterErrorx = vidcenterx - droguex
    CenterErrory = vidcentery - droguey

    xErrorWeight = abs(CenterErrorx) / vidcenterx
    yErrorWeight = abs(CenterErrory) / vidcentery

    totalError = xErrorWeight + yErrorWeight
    if totalError == 0:
        return [0.0, 0.0, 0.0, 0.0]

    maxErrorSide = max(xErrorWeight, yErrorWeight)
    ErrorWeightRatio = maxErrorSide / totalError

    if xErrorWeight > yErrorWeight:
        WeightDistancex = StraightDistancex * (1.0 - ErrorWeightRatio)
        WeightDistancey = StraightDistancey * ErrorWeightRatio
    elif xErrorWeight < yErrorWeight:
        WeightDistancex = StraightDistancex * ErrorWeightRatio
        WeightDistancey = StraightDistancey * (1.0 - ErrorWeightRatio)
    else:
        WeightDistancex = StraightDistancex * 0.5
        WeightDistancey = StraightDistancey * 0.5

    WeightedDistance = WeightDistancex + WeightDistancey

    if abs(CenterErrorx) < 1e-9 or abs(CenterErrory) < 1e-9:
        return [0.0, 0.0, float(WeightedDistance), float(WeightedDistance)]

    OffsetDistancex = WeightedDistance / (FocalLength / CenterErrorx)
    OffsetDistancey = WeightedDistance / (FocalLength / CenterErrory)

    CombinedOffset = np.hypot(OffsetDistancex, OffsetDistancey)
    FinalDistance = np.hypot(WeightedDistance, CombinedOffset)

    return [float(OffsetDistancex)* 0.0254, float(OffsetDistancey)* 0.0254, float(WeightedDistance)* 0.0254, float(FinalDistance)* 0.0254]


def apply_pc_and_norm(rng4, pc=PC):
    out = list(rng4)
    v = np.array(out[:3], dtype=float) + pc
    out[:3] = v.tolist()
    out[3] = float(np.linalg.norm(v))
    return out


# =========================
# DETECTION PICKING
# =========================
def pick_best_by_class(xyxy, confs, clss, class_id):
    idxs = np.where(clss == class_id)[0]
    if idxs.size == 0:
        return None
    best_i = idxs[np.argmax(confs[idxs])]
    x1, y1, x2, y2 = xyxy[best_i]
    return float(x1), float(y1), float(x2), float(y2), float(confs[best_i])


def draw_box(img, x1, y1, x2, y2, text, color, thickness=2):
    x1i, y1i, x2i, y2i = int(x1), int(y1), int(x2), int(y2)
    cv2.rectangle(img, (x1i, y1i), (x2i, y2i), color, thickness)
    cv2.putText(img, text, (x1i, max(0, y1i - 6)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)


# =========================
# MAIN
# =========================
def main():
    global STOP

    print("Loading YOLO model...")
    model = YOLO(MODEL_PATH)
    print("Model loaded.")

    # === UDP ===
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # optional: avoid blocking on send if something weird happens
    sock.setblocking(False)
    send_period = 1.0 / max(1e-6, UDP_SEND_HZ)
    t_last_send = 0.0

    print(f"Opening RTSP stream: {RTSP_URL}")
    cap = open_capture_rtsp(RTSP_URL)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open RTSP stream: {RTSP_URL}")

    t = threading.Thread(target=reader_thread, args=(cap,), daemon=True)
    t.start()

    last_seq_seen = -1
    frame_idx = 0
    t_last = time.time()

    if SHOW:
        cv2.namedWindow("DNN + Ranging (RTSP)", cv2.WINDOW_NORMAL)

    while True:
        with LOCK:
            seq = LATEST["seq"]
            frame = LATEST["frame"]
            ts = LATEST["ts"]

        if frame is None:
            time.sleep(0.005)
            continue

        if seq == last_seq_seen:
            time.sleep(0.001)
            if SHOW and cv2.waitKey(1) == 27:
                break
            continue
        last_seq_seen = seq

        h, w = frame.shape[:2]

        results = model.predict(
            source=frame,
            imgsz=IMGSZ,
            conf=CONF,
            iou=IOU,
            device=DEVICE,
            verbose=False
        )

        r = results[0]
        boxes = r.boxes

        annotated = frame.copy()
        coupler_out = [0.0] * 4
        drogue_out = [0.0] * 4
        coupler_conf = 0.0
        drogue_conf = 0.0
        have_coupler = False
        have_drogue = False

        if boxes is not None and len(boxes) > 0:
            xyxy = boxes.xyxy.detach().cpu().numpy()
            confs = boxes.conf.detach().cpu().numpy()
            clss = boxes.cls.detach().cpu().numpy().astype(int)

            coupler = pick_best_by_class(xyxy, confs, clss, CLASS_COUPLER)
            drogue = pick_best_by_class(xyxy, confs, clss, CLASS_DROGUE)

            if coupler is not None:
                x1c, y1c, x2c, y2c, cc = coupler
                coupler_conf = float(cc)
                have_coupler = True
                coupler_out = RangingFunction(x1c, y1c, x2c, y2c, n=1, frame_w=w, frame_h=h)
                coupler_out = apply_pc_and_norm(coupler_out, PC)

            if drogue is not None:
                x1d, y1d, x2d, y2d, dc = drogue
                drogue_conf = float(dc)
                have_drogue = True
                drogue_out = RangingFunction(x1d, y1d, x2d, y2d, n=2, frame_w=w, frame_h=h)
                drogue_out = apply_pc_and_norm(drogue_out, PC)

        snapshot_out = coupler_out + drogue_out
        # print(f"RANGE: {snapshot_out}")

        # === UDP SEND ===
        now = time.time()
        valid = bool(have_coupler or have_drogue)
        if (not UDP_SEND_ONLY_IF_VALID) or valid:
            if (now - t_last_send) >= send_period:
                pkt = {
                    "ts": now,          # sender wall time
                    "frame_ts": ts,     # capture time
                    "seq": int(seq),    # frame sequence from reader
                    "valid": valid,
                    "w": int(w),
                    "h": int(h),
                    "pc": [float(PC[0]), float(PC[1]), float(PC[2])],
                    "coupler": {
                        "valid": have_coupler,
                        "conf": coupler_conf,
                        "rng": [float(x) for x in coupler_out],  # [dx, dy, wdist, dist]
                    },
                    "drogue": {
                        "valid": have_drogue,
                        "conf": drogue_conf,
                        "rng": [float(x) for x in drogue_out],
                    },
                    # convenient combined vector if your control code expects it
                    "snapshot": [float(x) for x in snapshot_out],  # 8 values
                }
                try:
                    sock.sendto(json.dumps(pkt).encode("utf-8"), (UDP_IP, UDP_PORT))
                    t_last_send = now
                except (BlockingIOError, OSError):
                    # Non-fatal: just skip this send
                    pass

        if SHOW:
            cv2.imshow("DNN + Ranging (RTSP)", annotated)
            if cv2.waitKey(1) == 27:
                break

        frame_idx += 1

        if frame_idx % 30 == 0:
            fps = 30.0 / max(1e-6, (time.time() - t_last))
            lag_s = time.time() - ts
            # print(f"[INFO] infer FPS: {fps:.2f} | frame age: {lag_s:.3f}s")
            t_last = time.time()

    STOP = True
    try:
        cap.release()
    except Exception:
        pass
    try:
        cv2.destroyAllWindows()
    except Exception:
        pass


if __name__ == "__main__":
    try:
        main()
    finally:
        STOP = True
