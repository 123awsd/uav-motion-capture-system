import cv2
import numpy as np
import time
import json
from collections import deque
import signal
from pathlib import Path

# ---------------- 配置（不改相机任何属性） ----------------
THRESHOLD_VALUE = 249   # 亮点阈值
AREA_MAX = 3000         # 面积过滤上限
WARMUP_SECONDS = 1.0    # 打开后预热时间，给AE/AWB稳定
DRAW_TEXT_EVERY = 1     # 坐标文字绘制频率：1=每帧都画
# 你的内参结果文件（由 Intrinsic_calibration.py 生成）
INTRINSICS_JSON = "calibration_results/intrinsics_only.json"
# 将 /dev/video 索引映射到 JSON 中的 camera_id（按你保存时的命名来）
DEV_TO_CAMID = {0: "camera_0", 2: "camera_1", 4: "camera_2"}
# ------------------------------------------------------

KERNEL = np.ones((2, 2), np.uint8)

_shutdown = False
def _sigint(_sig, _frm):
    global _shutdown
    _shutdown = True
signal.signal(signal.SIGINT, _sigint)

# ---------------- 加载内参 ----------------
def load_intrinsics(json_path):
    p = Path(json_path)
    if not p.exists():
        print(f"[WARN] 未找到内参文件：{json_path}，将跳过畸变校正")
        return {}
    with p.open("r", encoding="utf-8") as f:
        data = json.load(f)
    intr = {}
    for cam_id, v in data.items():
        if cam_id == "_meta":
            continue
        K = np.array(v["K"], dtype=np.float32)
        dist = np.array(v["dist"], dtype=np.float32).reshape(1, -1)
        img_size = tuple(v.get("image_size", [0, 0]))  # [w, h]
        intr[cam_id] = {"K": K, "dist": dist, "image_size": img_size}
    print(f"[OK] 载入内参数量：{len(intr)}")
    return intr

INTR_DATA = load_intrinsics(INTRINSICS_JSON)

def scale_K(K, from_size_wh, to_size_wh):
    """把内参矩阵从标定分辨率缩放到当前分辨率。"""
    K = K.copy()
    fw, fh = from_size_wh
    tw, th = to_size_wh
    if fw <= 0 or fh <= 0 or (fw == tw and fh == th):
        return K
    sx = float(tw) / float(fw)
    sy = float(th) / float(fh)
    K[0, 0] *= sx; K[0, 2] *= sx
    K[1, 1] *= sy; K[1, 2] *= sy
    return K

def build_remap_for(dev_idx, frame_w, frame_h):
    """为当前设备与尺寸构建 undistort 的 remap（若无内参则返回 None）。"""
    cam_id = DEV_TO_CAMID.get(dev_idx)
    if not cam_id or cam_id not in INTR_DATA:
        return None
    K0   = INTR_DATA[cam_id]["K"]
    dist = INTR_DATA[cam_id]["dist"]
    cal_w, cal_h = INTR_DATA[cam_id]["image_size"] or (frame_w, frame_h)

    K = scale_K(K0, (cal_w, cal_h), (frame_w, frame_h))
    newK, _ = cv2.getOptimalNewCameraMatrix(K, dist, (frame_w, frame_h), alpha=0, newImgSize=(frame_w, frame_h))
    map1, map2 = cv2.initUndistortRectifyMap(K, dist, None, newK, (frame_w, frame_h), cv2.CV_16SC2)
    return map1, map2

def points_capture(imge):
    """
    从图像中提取高亮区域的中心点坐标，并在图像上标注出来。
    质心在【腐蚀后的图像】上计算，更准确。
    返回：points(list[(x,y)]), 带标注的图像
    """
    gray = cv2.cvtColor(imge, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, THRESHOLD_VALUE, 255, cv2.THRESH_BINARY)

    # 先腐蚀去噪，再膨胀仅用于连通与可视化（质心不用膨胀图）
    eroded  = cv2.erode(binary, KERNEL, iterations=1)
    dilated = cv2.dilate(eroded, KERNEL, iterations=6)

    # ✅ 在腐蚀后的图像上计算连通域与质心（更接近真实亮点）
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(eroded, connectivity=8)

    points = []
    if num_labels > 1:
        areas = stats[1:, cv2.CC_STAT_AREA]
        mask = areas < AREA_MAX
        valid_centroids = centroids[1:][mask]
        for (cx, cy) in valid_centroids:
            x, y = int(cx), int(cy)
            points.append((x, y))
            cv2.circle(imge, (x, y), 5, (0, 0, 255), -1)
            cv2.putText(imge, f'({x}, {y})', (x + 10, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    return points, imge

def camera_init(camera_indices=(0, 2, 4)):
    """
    仅打开摄像头（V4L2 后端），不改任何属性；打开后预热 WARMUP_SECONDS。
    返回：[(idx, cap), ...]
    """
    cams = []
    for idx in camera_indices:
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if not cap.isOpened():
            print(f"无法打开摄像头 {idx}")
            continue
        time.sleep(WARMUP_SECONDS)  # 与“正常显示”的程序一致
        print(f"✅ 摄像头 /dev/video{idx} 打开成功")
        cams.append((idx, cap))
    return cams

def main():
    cams = camera_init([0, 2, 4])
    if not cams:
        print("无可用摄像头，程序退出")
        return

    # 为每路/每尺寸缓存 remap
    remap_cache = {}

    t_hist = deque(maxlen=60)
    frame_counter = 0

    while not _shutdown:
        frame_counter += 1
        draw_text = (frame_counter % DRAW_TEXT_EVERY == 0)

        for dev_idx, cap in cams:
            ok, frame = cap.read()
            if not ok or frame is None:
                print(f"摄像头 /dev/video{dev_idx} 读取失败")
                continue

            h, w = frame.shape[:2]
            key = (dev_idx, w, h)
            if key not in remap_cache:
                maps = build_remap_for(dev_idx, w, h)
                remap_cache[key] = maps  # 可能为 None（无内参）
            maps = remap_cache[key]

            # === 仅新增：畸变校正（显示与点检测都基于校正后的图） ===
            if maps is not None:
                map1, map2 = maps
                und = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            else:
                und = frame
            # =======================================================

            vis = und.copy()
            points, vis = points_capture(vis)

            # 简单 HUD（不影响相机设置）
            if len(t_hist) >= 2:
                dt = (t_hist[-1] - t_hist[0]) if (t_hist[-1] - t_hist[0]) > 1e-6 else 1.0
                fps = (len(t_hist) - 1) / dt
            else:
                fps = 0.0
            hud = f"points: {len(points)} | fps: {fps:.1f}"
            cv2.putText(vis, hud, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 3, cv2.LINE_AA)
            cv2.putText(vis, hud, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)

            cv2.imshow(f"Camera /dev/video{dev_idx}", vis)

        t_hist.append(time.perf_counter())

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    for _, cap in cams:
        cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
