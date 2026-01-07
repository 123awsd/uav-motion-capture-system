
# -*- coding: utf-8 -*-
"""
transform.py（WITH_SCALE=True，输入中文、图像英文、控制台中文）
— 交互式采集：按下 s 采样一次（所有相机均检测到 1 个亮点时），随后在控制台输入该点在【目标世界坐标系】中的 (X Y Z)
— 支持：Umeyama/Kabsch 拟合（带尺度）、简易 RANSAC、统计输出、保存 transform.json

单位：mm
"""

import os
import json
from pathlib import Path
import numpy as np
import cv2
from typing import List, Dict

INTR_JSON = "calibration_results/intrinsics_only.json"
EXTR_JSON = "calibration_results/extrinsic_only.json"
SAVE_JSON = "calibration_results/transform.json"
CAMERA_INDICES = (0, 2, 4)

WITH_SCALE = True
RANSAC_ITERS = 300
RANSAC_INLIER_THRESH_MM = 20.0

try:
    from camera import camera_init, points_capture, DEV_TO_CAMID
    from localization import triangulate_point
except Exception as e:
    raise RuntimeError("请确保 camera.py 与 localization.py 在同一工程目录下") from e


def load_intrinsics(path: str) -> Dict[str, np.ndarray]:
    with open(path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    return {k: np.array(v["K"], float) for k, v in data.items() if k != "_meta"}


def load_extrinsics(path: str) -> Dict[str, Dict[str, np.ndarray]]:
    with open(path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    return {k: {"R": np.array(v["R"], float), "t": np.array(v["t"], float).reshape(3, 1)}
            for k, v in data.items() if k != "_meta"}


def build_pose_list(run_cam_ids, extr):
    return [{"R": extr[cid]["R"], "t": extr[cid]["t"]} for cid in run_cam_ids]


def umeyama_align(src, dst, with_scale=True, eps=1e-9):
    src = np.asarray(src, float)
    dst = np.asarray(dst, float)
    assert src.shape == dst.shape and src.shape[1] == 3 and src.shape[0] >= 3
    mu_src, mu_dst = src.mean(0), dst.mean(0)
    X, Y = src - mu_src, dst - mu_dst
    C = (X.T @ Y) / src.shape[0]
    U, S, Vt = np.linalg.svd(C)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    if with_scale:
        var = np.sum(X * X) / src.shape[0]
        s = S.sum() / (var + eps)
    else:
        s = 1.0
    t = mu_dst - s * (R @ mu_src)
    return s, R, t


def residuals_mm(s, R, t, src, dst):
    pred = (s * (R @ src.T)).T + t.reshape(1, 3)
    return np.linalg.norm(pred - dst, axis=1)


def ransac_umeyama(src, dst, with_scale, iters, thr_mm, min_sample=4):
    src = np.asarray(src, float)
    dst = np.asarray(dst, float)
    N = src.shape[0]
    if N < min_sample:
        raise ValueError("RANSAC至少需4点")
    rng = np.random.default_rng()
    best_in = None
    for _ in range(iters):
        idx = rng.choice(N, size=min_sample, replace=False)
        s, R, t = umeyama_align(src[idx], dst[idx], with_scale)
        res = residuals_mm(s, R, t, src, dst)
        inliers = res < thr_mm
        if best_in is None or inliers.sum() > best_in.sum():
            best_in = inliers
    if best_in.sum() < 3:
        s, R, t = umeyama_align(src, dst, with_scale)
        best_in = np.ones(N, bool)
    else:
        s, R, t = umeyama_align(src[best_in], dst[best_in], with_scale)
    return s, R, t, best_in


def run_transform_capture():
    if not (os.path.exists(INTR_JSON) and os.path.exists(EXTR_JSON)):
        raise FileNotFoundError("未找到内外参文件")
    Ks = load_intrinsics(INTR_JSON)
    extr = load_extrinsics(EXTR_JSON)
    cams = camera_init(CAMERA_INDICES)
    run_cam_ids = [DEV_TO_CAMID.get(dev, f"camera_{i}") for i, (dev, _) in enumerate(cams)]
    poses = build_pose_list(run_cam_ids, extr)
    intr = {cid: Ks[cid] for cid in run_cam_ids}
    print("=== 坐标系转换模式启动 ===")
    old_pts, tgt_pts = [], []
    try:
        while True:
            per_img, per_pts = [], []
            for (dev_idx, cap), cid in zip(cams, run_cam_ids):
                ok, frame = cap.read()
                if not ok:
                    per_img.append(None)
                    per_pts.append(None)
                    continue
                pts, img = points_capture(frame)
                per_img.append(img)
                per_pts.append(pts[0] if len(pts) == 1 else None)
            for (dev_idx, _), img in zip(cams, per_img):
                if img is None:
                    continue
                lines = [
                    f"/dev/video{dev_idx}",
                    f"Samples: {len(old_pts)}",
                    "Keys: s=Sample d=Delete c=Clear q=Finish h=Help"
                ]
                for i, ln in enumerate(lines):
                    cv2.putText(img, ln, (10, 26 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.imshow(f"Camera {dev_idx}", img)
                cv2.resizeWindow(f"Camera {dev_idx}", 640, 360)
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                break
            elif k == ord('s'):
                if any(p is None for p in per_pts):
                    print("采样失败，有相机未检测到点")
                    continue
                img_pts = np.array(per_pts, float)
                X_old = triangulate_point(img_pts.tolist(), poses, intr)
                txt = input("请输入该点在【目标世界】中的坐标 X Y Z（以空格分隔，单位mm）：\n> ").strip()
                vals = [float(v) for v in txt.replace(',', ' ').split() if v]
                if len(vals) != 3:
                    print("输入格式错误")
                    continue
                X_tgt = np.array(vals, float)
                old_pts.append(X_old)
                tgt_pts.append(X_tgt)
                print(f"✓ 已采样第{len(old_pts)}组：旧世界{X_old}→目标世界{X_tgt}")
            elif k == ord('d') and old_pts:
                old_pts.pop()
                tgt_pts.pop()
                print("删除上一组")
            elif k == ord('c'):
                old_pts.clear()
                tgt_pts.clear()
                print("已清空样本")
    finally:
        for _, cap in cams:
            cap.release()
        cv2.destroyAllWindows()

    if len(old_pts) < 4:
        print("样本数不足，至少4组")
        return

    src = np.vstack(old_pts)
    dst = np.vstack(tgt_pts)
    s, R, t, inliers = ransac_umeyama(src, dst, WITH_SCALE, RANSAC_ITERS, RANSAC_INLIER_THRESH_MM)
    res = residuals_mm(s, R, t, src[inliers], dst[inliers])
    stats = {
        "n_total": int(src.shape[0]),
        "n_inliers": int(inliers.sum()),
        "s": float(s),
        "rms_mm": float(np.sqrt(np.mean(res ** 2)))
    }
    U, _, Vt = np.linalg.svd(R)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        R = -R
    Path(os.path.dirname(SAVE_JSON) or ".").mkdir(parents=True, exist_ok=True)
    out = {
        "convention": "old_world_to_target_world",
        "with_scale": True,
        "s": float(s),
        "R": R.tolist(),
        "t": t.reshape(3).tolist(),
        "stats": stats
    }
    with open(SAVE_JSON, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print(f"[OK] 已保存 {SAVE_JSON}")
    print(f"s={s:.6f}\nR=\n{R}\nt={t.reshape(3)}\n统计:{stats}")


if __name__ == "__main__":
    run_transform_capture()
