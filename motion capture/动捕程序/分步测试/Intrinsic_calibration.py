# -*- coding: utf-8 -*-
"""
Intrinsic_calibration.py — 多相机内参标定（K + dist，覆盖筛选关闭 + 点/图两级剔除）
默认棋盘：11×8 内角点，每格 25mm

特性：
  - 5 参数畸变模型（k1,k2,p1,p2,k3）
  - 自适应模糊阈值（Laplacian 方差 p20 分位）
  - 角点检测优先使用 ChessboardSB，失败回退经典接口
  - 按分辨率分组：仅使用样本最多的那一组进行标定
  - 两级剔除：
      * 点级：按重投影误差阈值（默认 3px）剔除离群角点
      * 图级：按平均误差保留最佳 70%（可调）且至少 12 张（可调）
  - 打印详细日志（统计与误差分布），输出被剔除样本清单（Top-10）

使用：
  直接运行（默认从 data/camera_0,1,2 读取）：
    python Intrinsic_calibration.py

  或命令行自定义：
    python Intrinsic_calibration.py \
      --cam camera_0:data/camera_0 --cam camera_1:data/camera_1 --cam camera_2:data/camera_2 \
      --cols 11 --rows 8 --square 25 \
      --keep-ratio 0.7 --min-keep 12 --sb-only 0 \
      --out calibration_results/intrinsics_only.json
"""
from __future__ import annotations
import argparse
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Tuple

import cv2
import numpy as np
from mytool import save_to_json  # 与工程内 mytool.py 一致


# ===== 默认参数 =====
DEFAULT_CHESS_COLS = 11
DEFAULT_CHESS_ROWS = 8
DEFAULT_SQUARE_MM = 20.0#mm

DEFAULT_CAM_DIRS = {
    "camera_0": Path("data/camera_0"),
    "camera_1": Path("data/camera_1"),
    "camera_2": Path("data/camera_2"),
}

# ===== I/O & 小工具 =====
def _glob_images(folder: Path) -> List[str]:
    exts = ("*.png", "*.jpg", "*.jpeg", "*.bmp", "*.tif", "*.tiff")
    files: List[str] = []
    for e in exts:
        files.extend([str(p) for p in folder.glob(e)])
    files.sort()
    return files

def _lap_var(img_gray: np.ndarray) -> float:
    """Laplacian 方差：数值越大越清晰。"""
    return float(cv2.Laplacian(img_gray, cv2.CV_64F).var())

def _per_view_err(K, dist, rvecs, tvecs, objpoints, imgpoints) -> List[float]:
    errs = []
    for obj, img, rvec, tvec in zip(objpoints, imgpoints, rvecs, tvecs):
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        e = np.mean(np.linalg.norm(proj.reshape(-1, 2) - img.reshape(-1, 2), axis=1))
        errs.append(float(e))
    return errs

def _find_corners(im: np.ndarray, pattern_size: Tuple[int, int], term, sb_only: bool) -> Tuple[bool, np.ndarray]:
    """优先 ChessboardSB，失败时回退经典接口（除非 sb_only=True）。"""
    # 先试 SB（更稳）
    ret, corners = cv2.findChessboardCornersSB(im, pattern_size, flags=cv2.CALIB_CB_EXHAUSTIVE)
    if ret:
        corners = cv2.cornerSubPix(im, corners.astype(np.float32), (11, 11), (-1, -1), term)
        return True, corners
    if sb_only:
        return False, None

    # 回退经典方法
    ret, corners = cv2.findChessboardCorners(
        im, pattern_size,
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    )
    if ret:
        corners = cv2.cornerSubPix(im, corners, (11, 11), (-1, -1), term)
    return ret, corners

def _filter_points_by_reproj(K, dist, rvecs, tvecs, objpoints, imgpoints, thr_px: float = 3.0):
    """点级剔除：每幅图按重投影误差阈值筛角点；至少保留一半角点才保留这一幅。"""
    obj2, img2 = [], []
    for obj, img, rvec, tvec in zip(objpoints, imgpoints, rvecs, tvecs):
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        proj = proj.reshape(-1, 2)
        img2d = img.reshape(-1, 2)
        err = np.linalg.norm(proj - img2d, axis=1)
        mask = err < thr_px
        if mask.sum() >= max(10, int(0.5 * len(err))):  # 至少保留一半或>=10个角点
            obj2.append(obj[mask])
            img2.append(img[mask].reshape(-1, 1, 2))
    return obj2, img2


# ===== 单相机标定主流程 =====
def calibrate_single_camera(
    image_paths: List[str],
    cols: int,
    rows: int,
    square: float,
    keep_ratio: float = 0.70,
    min_keep: int = 12,
    sb_only: bool = False,
    point_thr_px: float = 3.0,
) -> Dict:
    """
    返回：
      {
        "camera_matrix": K(np.ndarray 3x3),
        "dist_coeffs": dist(np.ndarray 1x5),
        "rms": float,
        "image_size": (w, h),
        "num_views": int
      }
    """
    assert 0 < keep_ratio <= 1.0, "keep_ratio 必须在 (0,1] 范围"
    assert min_keep >= 3, "min_keep 至少 3"

    pattern_size = (cols, rows)
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= float(square)
    term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)

    # 1) 读取图像，统计 Laplacian 方差，计算自适应阈值（p20）
    imgs_gray, lap_vars = [], []
    for p in image_paths:
        im = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        if im is None:
            continue
        imgs_gray.append((p, im))
        lap_vars.append(_lap_var(im))
    if not imgs_gray:
        raise RuntimeError("未读取到任何有效图像。请检查路径/格式。")

    thr = float(np.percentile(lap_vars, 20))
    lv_min, lv_median = float(np.min(lap_vars)), float(np.median(lap_vars))
    print(f"[INFO] LapVar: min={lv_min:.1f}  p20={thr:.1f}  median={lv_median:.1f}  (自适应模糊阈值=p20)")

    # 2) 角点检测（优先 SB），记录路径/尺寸
    objpoints, imgpoints, sizes, paths_kept = [], [], [], []
    n_total = len(imgs_gray)
    n_blur = n_nocb = 0

    for (p, im), lv in zip(imgs_gray, lap_vars):
        if lv < thr:
            n_blur += 1
            continue
        h, w = im.shape[:2]
        ret, corners = _find_corners(im, pattern_size, term, sb_only=sb_only)
        if not ret:
            n_nocb += 1
            continue
        objpoints.append(objp.copy())
        imgpoints.append(corners)
        sizes.append((w, h))
        paths_kept.append(p)

    if not objpoints:
        raise RuntimeError("模糊/角点检测过滤后仍为 0；请检查棋盘规格/曝光/清晰度或降低 sb_only。")

    # 3) 按分辨率分组，选择样本数最多的组
    buckets = defaultdict(lambda: {"obj": [], "img": [], "paths": []})
    for sz, obj, img, p in zip(sizes, objpoints, imgpoints, paths_kept):
        buckets[sz]["obj"].append(obj)
        buckets[sz]["img"].append(img)
        buckets[sz]["paths"].append(p)

    image_size = max(buckets.keys(), key=lambda k: len(buckets[k]["obj"]))
    objpoints = buckets[image_size]["obj"]
    imgpoints = buckets[image_size]["img"]
    paths_kept = buckets[image_size]["paths"]

    print(f"[INFO] 进入标定的分辨率：{image_size}  样本数：{len(objpoints)}")

    # 4) 第一次标定（5 参模型）
    flags = 0  # 不启用 RATIONAL_MODEL，使用 k1,k2,p1,p2,k3
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, image_size, None, None, flags=flags
    )

    # 5) 点级离群剔除（可提升拟合稳定性）
    obj_f, img_f = _filter_points_by_reproj(K, dist, rvecs, tvecs, objpoints, imgpoints, thr_px=point_thr_px)
    if len(obj_f) >= max(min_keep, int(0.5 * len(objpoints))):
        objpoints, imgpoints = obj_f, img_f
        rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, image_size, None, None, flags=flags
        )

    # 6) 计算每幅图平均误差
    per_view = _per_view_err(K, dist, rvecs, tvecs, objpoints, imgpoints)
    N = len(per_view)
    used_after = N

    # 7) 图级二次剔除：保留最佳 keep_ratio，至少 min_keep
    if N >= min_keep and keep_ratio < 1.0:
        keep_n = max(min_keep, int(round(N * keep_ratio)))
        idx_sorted = np.argsort(per_view)  # 小→大
        keep_idx = set(idx_sorted[:keep_n])

        # 打印将被剔除的样本（Top-10）
        dropped = [(paths_kept[i], per_view[i]) for i in range(N) if i not in keep_idx]
        if dropped:
            dropped.sort(key=lambda x: x[1], reverse=True)  # 误差大的在前
            print("[INFO] 将剔除的样本（Top-10，按误差降序）：")
            for p, e in dropped[:10]:
                print(f"  [DROP] {p}  err={e:.2f}px")

        obj2 = [objpoints[i] for i in range(N) if i in keep_idx]
        img2 = [imgpoints[i] for i in range(N) if i in keep_idx]
        used_after = len(obj2)

        rms2, K2, dist2, rvecs2, tvecs2 = cv2.calibrateCamera(
            obj2, img2, image_size, None, None, flags=flags
        )
        # 若更优或样本更多，则采用
        if (rms2 < rms) or (len(obj2) > len(objpoints)):
            rms, K, dist = rms2, K2, dist2
            # 重新统计 per-view（用于日志）
            per_view = _per_view_err(K, dist, rvecs2, tvecs2, obj2, img2)

    # 8) 形状统一、日志输出
    dist = dist.reshape(1, -1)
    pv_min = float(np.min(per_view)) if per_view else float("nan")
    pv_med = float(np.median(per_view)) if per_view else float("nan")
    pv_p95 = float(np.percentile(per_view, 95)) if per_view else float("nan")

    print(f"[INFO] 总:{n_total}  模糊:{n_blur}  无棋盘:{n_nocb}  进入分组:{len(paths_kept)}  用于二次优化:{used_after} / 初始:{N}")
    print(f"[INFO] rms={rms:.3f}px  dist_len={dist.size}  size={image_size}  per-view: min={pv_min:.2f}  median={pv_med:.2f}  p95={pv_p95:.2f}")

    return {
        "camera_matrix": K,
        "dist_coeffs": dist,
        "rms": float(rms),
        "image_size": image_size,
        "num_views": int(used_after),
    }


# ===== 多相机入口 =====
def save_intrinsics_for_multi_cameras(
    cameras_images: Dict[str, List[str]],
    cols: int,
    rows: int,
    square: float,
    save_path: str,
    keep_ratio: float = 0.70,
    min_keep: int = 12,
    sb_only: bool = False,
    point_thr_px: float = 3.0,
):
    results = {
        "_meta": {
            "cols": cols,
            "rows": rows,
            "square_mm": square,
            "keep_ratio": keep_ratio,
            "min_keep": min_keep,
            "sb_only": int(sb_only),
        }
    }

    for cam_id, imgs in cameras_images.items():
        if not imgs:
            print(f"[WARN] {cam_id} 无图片，跳过")
            continue
        print(f"\n[INFO] 标定 {cam_id}，共 {len(imgs)} 张图...")
        res = calibrate_single_camera(
            imgs, cols, rows, square,
            keep_ratio=keep_ratio, min_keep=min_keep,
            sb_only=sb_only, point_thr_px=point_thr_px,
        )
        results[cam_id] = {
            "K": res["camera_matrix"].tolist(),
            "dist": np.asarray(res["dist_coeffs"]).ravel().tolist(),  # 扁平写入
            "rms": res["rms"],
            "image_size": list(res["image_size"]),
            "num_views": int(res["num_views"]),
        }

    Path(save_path).parent.mkdir(parents=True, exist_ok=True)
    save_to_json(results, save_path)
    print(f"\n[OK] 内参已保存：{save_path}")


# ===== CLI =====
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="多相机内参标定（11x8，每格25mm；SB优先 + 点/图两级剔除）")
    parser.add_argument("--cam", action="append", default=[],
                        help="相机与路径，如 camera_0:data/camera_0，可多次使用")
    parser.add_argument("--cols", type=int, default=DEFAULT_CHESS_COLS)
    parser.add_argument("--rows", type=int, default=DEFAULT_CHESS_ROWS)
    parser.add_argument("--square", type=float, default=DEFAULT_SQUARE_MM)
    parser.add_argument("--keep-ratio", type=float, default=0.70,
                        help="图级二次剔除保留比例，默认 0.70")
    parser.add_argument("--min-keep", type=int, default=12,
                        help="图级二次剔除最少保留张数，默认 12")
    parser.add_argument("--sb-only", type=int, default=0,
                        help="仅使用 ChessboardSB 检测（1 开启 / 0 关闭，默认 0）")
    parser.add_argument("--point-thr", type=float, default=3.0,
                        help="点级剔除的重投影误差阈值（px），默认 3.0")
    parser.add_argument("--out", type=str, default="calibration_results/intrinsics_only.json")
    return parser.parse_args()

def main_cli():
    args = parse_args()

    cams: Dict[str, List[str]] = {}
    if args.cam:
        for item in args.cam:
            if ":" not in item:
                raise ValueError(f"--cam 参数格式错误: {item}")
            cam_id, folder = item.split(":", 1)
            imgs = _glob_images(Path(folder))
            if not imgs:
                print(f"[WARN] {cam_id} 在 {folder} 未找到图片")
            cams[cam_id] = imgs
    else:
        for cam_id, folder in DEFAULT_CAM_DIRS.items():
            imgs = _glob_images(folder)
            if imgs:
                cams[cam_id] = imgs

    if not cams:
        raise RuntimeError("未找到任何图片，请检查 data/camera_x 文件夹或使用 --cam 指定。")

    save_intrinsics_for_multi_cameras(
        cams,
        cols=args.cols,
        rows=args.rows,
        square=args.square,
        save_path=args.out,
        keep_ratio=args.keep_ratio,
        min_keep=args.min_keep,
        sb_only=bool(args.sb_only),
        point_thr_px=args.point_thr,
    )

if __name__ == "__main__":
    main_cli()
