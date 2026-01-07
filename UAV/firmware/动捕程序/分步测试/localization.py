
# -*- coding: utf-8 -*-
"""
Localization (revised)
- Triangulate a single bright point from multiple cameras
- Optionally rebase extrinsics to a target world frame using a transform.json
- Or, without rebasing, apply the transform to triangulated points before display/save
- Shows per-camera status, 3D point, reprojection error, and writes CSV on demand

Hotkeys: q=quit | s=toggle CSV save | h=help
"""

import os
import json
import time
import csv
from pathlib import Path
from typing import List, Dict, Optional, Tuple

import numpy as np
import cv2

# ========= Config =========
INTR_JSON = "calibration_results/intrinsics_only.json"
EXTR_JSON = "calibration_results/extrinsic_only.json"
TRANSFORM_JSON_CANDIDATES = [
    "calibration_results/transform.json",      # preferred name
    "calibration_results/transfrom.json",     # misspelling fallback
]
CAMERA_INDICES = (0, 2, 4)          # your /dev/video* indices
SAVE_CSV = "calibration_results/live_points.csv"

# How to output coordinates:
#   "old_world"    -> the world used in extrinsic calibration (default == cam0 world)
#   "target_world" -> a desired world frame defined by transform.json
OUTPUT_FRAME = "target_world"

# Whether to rebase extrinsics to the target world (so triangulation is already in target world)
# If False, we triangulate in old world and then map X_old -> X_target using the transform per-point.
REBASE_EXTRINSICS_TO_TARGET = True


# ========= Project Integration: Camera & Point Capture =========
try:
    from camera import camera_init, points_capture, DEV_TO_CAMID
except Exception as e:
    raise RuntimeError("Cannot import camera.py; ensure it is in the same project directory") from e


# =================== Projection utilities ===================

def make_projection_matrix(K: np.ndarray, R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """
    P = K [R|t], using world->camera extrinsics: X_c = R * X_w + t
    """
    t = t.reshape(3, 1)
    Rt = np.hstack([R, t])
    return K @ Rt


def project_points_world_to_image(Xw: np.ndarray,
                                  K: np.ndarray, R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """
    Project Nx3 world points to image plane. Returns Nx2.
    """
    Xw = np.asarray(Xw, dtype=np.float64).reshape(-1, 3)
    rvec, _ = cv2.Rodrigues(R.astype(np.float64))
    img_pts, _ = cv2.projectPoints(Xw, rvec, t.reshape(3, 1), K.astype(np.float64), None)
    return img_pts.reshape(-1, 2)


# =================== Triangulation ===================

def _triangulate_dlt(points_2d: List[Tuple[float, float]], P_list: List[np.ndarray]) -> np.ndarray:
    """
    Linear DLT triangulation from multiple views.
    """
    A = []
    for (u, v), P in zip(points_2d, P_list):
        p1, p2, p3 = P[0], P[1], P[2]
        A.append(u * p3 - p1)
        A.append(v * p3 - p2)
    A = np.asarray(A, dtype=np.float64)
    _, _, Vt = np.linalg.svd(A)
    Xh = Vt[-1]
    X = Xh[:3] / Xh[3]
    return X.astype(np.float64)


def triangulate_point(image_points: List[List[float]],
                      camera_poses: List[Dict[str, np.ndarray]],
                      intrinsics: Dict[str, np.ndarray]) -> np.ndarray:
    """
    Recover a single 3D world point from per-camera pixel coords.
    image_points: [[u0,v0], [u1,v1], ...] (same order as camera_poses)
    camera_poses: [{'R':3x3, 't':3x1}, ...] (world->camera extrinsics)
    intrinsics: {'camera_i': K_i}
    returns: 3-vector in the *current world* used by camera_poses
    """
    P_list = []
    for i, pose in enumerate(camera_poses):
        K = intrinsics[f"camera_{i}"]
        P_list.append(make_projection_matrix(K, pose['R'], pose['t']))
    X = _triangulate_dlt([tuple(p) for p in image_points], P_list)
    return X


def triangulate_points(image_points_sets: np.ndarray,
                       camera_poses: List[Dict[str, np.ndarray]],
                       intrinsics: Dict[str, np.ndarray]) -> List[Optional[np.ndarray]]:
    """
    Batch triangulation: image_points_sets shape (N_points, N_cams, 2)
    """
    N, C, _ = image_points_sets.shape
    P_list = [make_projection_matrix(intrinsics[f"camera_{i}"], p['R'], p['t']) for i, p in enumerate(camera_poses)]
    results = []
    for n in range(N):
        pts = [tuple(image_points_sets[n, i]) for i in range(C)]
        try:
            X = _triangulate_dlt(pts, P_list)
            if np.any(np.isnan(X)) or np.any(~np.isfinite(X)):
                results.append(None)
            else:
                results.append(X)
        except Exception:
            results.append(None)
    return results


# =================== Reprojection error ===================

def calculate_reprojection_error(image_points: np.ndarray,
                                 object_point: np.ndarray,
                                 camera_poses: List[Dict[str, np.ndarray]],
                                 intrinsics: Dict[str, np.ndarray]) -> Optional[float]:
    """
    Mean absolute pixel error across all cameras for a single 3D point.
    """
    if object_point is None or np.any(~np.isfinite(object_point)):
        return None
    errs = []
    for i in range(image_points.shape[0]):
        K = intrinsics.get(f"camera_{i}")
        if K is None:
            continue
        R = camera_poses[i]['R']
        t = camera_poses[i]['t']
        rvec, _ = cv2.Rodrigues(R.astype(np.float64))
        uv_pred, _ = cv2.projectPoints(object_point.reshape(1, 3), rvec, t.reshape(3, 1), K, None)
        uv_pred = uv_pred.reshape(2)
        uv_obs = image_points[i]
        errs.append(np.linalg.norm(uv_pred - uv_obs))
    if not errs:
        return None
    return float(np.mean(errs))


def calculate_reprojection_errors(image_points_sets: np.ndarray,
                                  object_points: List[Optional[np.ndarray]] or np.ndarray,
                                  camera_poses: List[Dict[str, np.ndarray]],
                                  intrinsics: Dict[str, np.ndarray]) -> np.ndarray:
    """
    Mean pixel reprojection error for each point.
    """
    N = image_points_sets.shape[0]
    if isinstance(object_points, list):
        Xs = np.array([p if p is not None else [np.nan, np.nan, np.nan] for p in object_points], dtype=np.float64)
    else:
        Xs = np.asarray(object_points, dtype=np.float64)
    errs = []
    for n in range(N):
        if np.any(~np.isfinite(Xs[n])):
            continue
        e = calculate_reprojection_error(image_points_sets[n], Xs[n], camera_poses, intrinsics)
        if e is not None:
            errs.append(e)
    return np.array(errs, dtype=np.float64)


def reprojection_summary(errors: np.ndarray) -> Dict[str, float]:
    if errors.size == 0:
        return {"mean_abs_px": 0.0, "median_L2_px": 0.0, "p95_L2_px": 0.0}
    return {
        "mean_abs_px": float(np.mean(errors)),
        "median_L2_px": float(np.median(errors)),
        "p95_L2_px": float(np.percentile(errors, 95)),
    }


# =================== I/O helpers ===================

def load_intrinsics(path: str) -> Dict[str, np.ndarray]:
    with open(path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    Ks = {}
    for k, v in data.items():
        if k == "_meta":
            continue
        Ks[k] = np.array(v["K"], dtype=np.float64)
    return Ks


def load_extrinsics(path: str) -> Dict[str, Dict[str, np.ndarray]]:
    with open(path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    extr = {}
    for k, v in data.items():
        if k == "_meta":
            continue
        extr[k] = {"R": np.array(v["R"], dtype=np.float64),
                   "t": np.array(v["t"], dtype=np.float64).reshape(3, 1)}
    meta = data.get("_meta", {})
    return extr, meta


def build_pose_list_by_run_order(run_cam_ids: List[str],
                                 extr: Dict[str, Dict[str, np.ndarray]]) -> List[Dict[str, np.ndarray]]:
    poses = []
    for cid in run_cam_ids:
        if cid not in extr:
            raise RuntimeError(f"Extrinsics missing {cid}")
        poses.append({"R": extr[cid]["R"], "t": extr[cid]["t"]})
    return poses


def _camera_centers_from_extrinsics(extr: Dict[str, Dict[str, np.ndarray]]):
    centers = {}
    for cid, v in extr.items():
        R = v["R"]
        t = v["t"].reshape(3, 1)
        C = -R.T @ t  # camera center in world
        centers[cid] = C.reshape(3)
    return centers


# =================== World transform handling ===================

def _load_transform_json(candidates: List[str]):
    for p in candidates:
        if os.path.exists(p):
            with open(p, "r", encoding="utf-8") as f:
                data = json.load(f)
            return data, p
    return None, None


def _parse_transform(data: dict) -> Tuple[np.ndarray, np.ndarray, float, str]:
    """
    Accepts a transform.json that may contain:
      - "R": 3x3, "t": 3, optional "s": float
      - "convention": string, recommended "old_world_to_target_world"
    Returns (Rw, tw, s, convention)
    X_target = s * Rw * X_old + tw
    """
    R = np.array(data.get("R", np.eye(3)), dtype=np.float64)
    t = np.array(data.get("t", [0, 0, 0]), dtype=np.float64).reshape(3)
    s = float(data.get("s", 1.0))
    convention = str(data.get("convention", "old_world_to_target_world"))
    # sanitize rotation to be orthonormal (in case of numeric noise)
    U, _, Vt = np.linalg.svd(R)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        R = -R
    return R, t, s, convention


def _rebase_extrinsics_to_target_world(extr_old: Dict[str, Dict[str, np.ndarray]],
                                       Rw: np.ndarray, tw: np.ndarray, s: float = 1.0) -> Dict[str, Dict[str, np.ndarray]]:
    """
    Given old-world->camera extrinsics (Ri, ti), and target world defined by:
        X_target = s * Rw * X_old + tw
    New extrinsics (target-world->camera) satisfy:
        X_c = R_i * X_old + t_i = R_i * ( (1/s) * Rw^T * (X_target - tw) ) + t_i
    =>  R'_i = (1/s) * R_i * Rw^T
        t'_i = t_i - (1/s) * R_i * Rw^T * tw
    Notes:
        - If s != 1, new "R'" is no longer a pure rotation. For typical use, keep s=1 here,
          and ensure metric scaling is handled during extrinsic calibration, not here.
    """
    RwT = Rw.T
    tw = tw.reshape(3, 1)
    new_extr = {}
    for cid, v in extr_old.items():
        Ri = np.asarray(v["R"], float)
        ti = np.asarray(v["t"], float).reshape(3, 1)
        Rn = (Ri @ RwT) / s
        tn = ti - (Ri @ RwT @ tw) / s
        new_extr[cid] = {"R": Rn, "t": tn}
    return new_extr


def _apply_point_transform_old_to_target(X_old: np.ndarray, Rw: np.ndarray, tw: np.ndarray, s: float = 1.0) -> np.ndarray:
    """
    X_target = s * Rw * X_old + tw
    """
    return (s * (Rw @ X_old.reshape(3, 1))).reshape(3) + tw.reshape(3)


# =================== overlay / UI ===================

def overlay_text(img, lines, org=(10, 30)):
    y = org[1]
    for line in lines:
        cv2.putText(img, line, (org[0], y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
        y += 22


# =================== Main loop ===================

def live_localization():
    # Check calibration files
    if not os.path.exists(INTR_JSON) or not os.path.exists(EXTR_JSON):
        raise FileNotFoundError("Please complete calibration first: intrinsics_only.json and extrinsic_only.json not found.")

    Ks = load_intrinsics(INTR_JSON)
    print(f"[OK] Intrinsics loaded: {len(Ks)} cameras")
    extr_old, meta_old = load_extrinsics(EXTR_JSON)
    print(f"[OK] Extrinsics loaded: {len(extr_old)} (file: {EXTR_JSON})")

    # Load optional transform to target world
    Rw = np.eye(3)
    tw = np.zeros(3)
    s_w = 1.0
    transform_path = None
    if OUTPUT_FRAME == "target_world":
        data, transform_path = _load_transform_json(TRANSFORM_JSON_CANDIDATES)
        if data is None:
            print("[WARN] OUTPUT_FRAME=target_world but no transform.json found. Proceeding without transform.")
        else:
            Rw, tw, s_w, convention = _parse_transform(data)
            print(f"[OK] Transform loaded from: {transform_path}")
            print(f"     convention: {convention}, s={s_w:.6f}")
            if abs(s_w - 1.0) > 1e-9 and REBASE_EXTRINSICS_TO_TARGET:
                print("[WARN] Non-unit scale found in transform. For clean rotations, prefer metric scaling during extrinsics calibration.")
            # Proceed anyway

    # Prepare camera devices
    cams = camera_init(CAMERA_INDICES)  # returns [(dev_idx, cap), ...]
    if not cams:
        print("No camera opened")
        return

    for dev_idx, _ in cams:
        print(f"✅ Camera /dev/video{dev_idx} opened")

    run_cam_ids = [DEV_TO_CAMID.get(dev, f"camera_{i}") for i, (dev, _) in enumerate(cams)]
    # Check all needed intr/extr exist
    for cid in run_cam_ids:
        if cid not in Ks:
            raise RuntimeError(f"Missing intrinsics: {cid}")
        if cid not in extr_old:
            raise RuntimeError(f"Missing extrinsics: {cid}")

    # Decide extrinsics used for triangulation (world->camera)
    if OUTPUT_FRAME == "target_world" and REBASE_EXTRINSICS_TO_TARGET and transform_path is not None:
        extr = _rebase_extrinsics_to_target_world(extr_old, Rw, tw, s=s_w)
        current_world_name = "target_world"
        print("[INFO] Extrinsics rebased to target world.")
    else:
        extr = extr_old
        current_world_name = "old_world (cam0-based)"

    # Build poses and intrinsics dict in run order
    poses = build_pose_list_by_run_order(run_cam_ids, extr)
    intr = {cid: Ks[cid] for cid in run_cam_ids}

    # Log mapping and baselines
    print("[Mapping] run order:")
    for (dev, _), cid in zip(cams, run_cam_ids):
        print(f"    /dev/video{dev} -> {cid}")
    print(f"[Convention] Using world->camera extrinsics: X_c = R * X_w + t; triangulation world = {current_world_name}")
    centers = _camera_centers_from_extrinsics(extr)
    for cid in run_cam_ids:
        C = centers[cid]
        print(f"    {cid} center C_w = ({C[0]:.3f}, {C[1]:.3f}, {C[2]:.3f})")
    if len(run_cam_ids) >= 2:
        for i in range(len(run_cam_ids) - 1):
            a, b = run_cam_ids[i], run_cam_ids[i + 1]
            d = np.linalg.norm(centers[a] - centers[b])
            print(f"    baseline |{a}-{b}| = {d:.3f} (world units)")

    # Runtime
    saving = False
    csv_file = None
    csv_writer = None
    last = time.time()
    fps = 0.0

    print("Hotkeys: q=quit | s=save/stop CSV | h=help")
    try:
        while True:
            per_img = []
            per_pts = []  # [(u,v) or None]

            # per-camera capture
            for (dev_idx, cap), cid in zip(cams, run_cam_ids):
                ok, frame = cap.read()
                if not ok or frame is None:
                    per_img.append(None)
                    per_pts.append(None)
                    continue
                pts, annotated = points_capture(frame)  # pts: [(x,y), ...]
                if len(pts) == 1:
                    per_pts.append(pts[0])
                else:
                    per_pts.append(None)
                per_img.append(annotated)

            # FPS
            now = time.time()
            inst_fps = (1.0 / (now - last) if now > last else 0.0)
            fps = 0.9 * fps + 0.1 * inst_fps
            last = now

            # Triangulate (in whichever world the current 'poses' use)
            valid = all(p is not None for p in per_pts) and len(per_pts) == len(run_cam_ids)
            Xw_cur = None
            mean_err = None
            if valid:
                img_pts = np.array(per_pts, dtype=np.float64)  # (C,2)
                Xw_cur = triangulate_point(img_pts.tolist(), poses, intr)
                mean_err = calculate_reprojection_error(img_pts, Xw_cur, poses, intr)

            # Optionally map to target world if we didn't rebase extrinsics
            Xw_display = None
            frame_label = current_world_name
            if Xw_cur is not None:
                if OUTPUT_FRAME == "target_world" and not (REBASE_EXTRINSICS_TO_TARGET and transform_path is not None) and transform_path is not None:
                    Xw_display = _apply_point_transform_old_to_target(Xw_cur, Rw, tw, s=s_w)
                    frame_label = "target_world (mapped)"
                else:
                    Xw_display = Xw_cur

            # Display
            for (dev_idx, _), img, idx in zip(cams, per_img, range(len(run_cam_ids))):
                if img is None:
                    continue
                cid = run_cam_ids[idx]
                lines = [f"/dev/video{dev_idx}  {cid}", f"FPS: {fps:.1f}"]
                p = per_pts[idx]
                if p is None:
                    lines.append("Point: --")
                else:
                    lines.append(f"Point: ({p[0]:.1f},{p[1]:.1f})")
                if Xw_display is not None:
                    lines.append(f"X[{frame_label}]: ({Xw_display[0]:.3f}, {Xw_display[1]:.3f}, {Xw_display[2]:.3f})")
                    if mean_err is not None:
                        lines.append(f"Reproj mean px: {mean_err:.3f}")
                overlay_text(img, lines, (10, 25))
                win = f"Cam {dev_idx}"
                cv2.namedWindow(win, cv2.WINDOW_NORMAL)
                cv2.imshow(win, img)
                cv2.resizeWindow(win, 640, 360)

            # CSV save
            if saving and Xw_display is not None:
                if csv_file is None:
                    Path(os.path.dirname(SAVE_CSV) or ".").mkdir(parents=True, exist_ok=True)
                    csv_file = open(SAVE_CSV, "w", newline="", encoding="utf-8")
                    csv_writer = csv.writer(csv_file)
                    header = ["timestamp", "frame", "X", "Y", "Z", "reproj_mean_px"]
                    for cid in run_cam_ids:
                        header += [f"{cid}_u", f"{cid}_v"]
                    csv_writer.writerow(header)
                row = [time.time(), frame_label,
                       Xw_display[0], Xw_display[1], Xw_display[2],
                       (mean_err if mean_err is not None else "")]
                for p in per_pts:
                    if p is None:
                        row += ["", ""]
                    else:
                        row += [p[0], p[1]]
                csv_writer.writerow(row)

            # Keys
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                break
            elif k == ord('s'):
                saving = not saving
                print("CSV:", "ON -> " + SAVE_CSV if saving else "OFF")
                if not saving and csv_file is not None:
                    csv_file.close()
                    csv_file = None
                    csv_writer = None
            elif k == ord('h'):
                print("q=quit, s=save/stop CSV, h=help")

    finally:
        if 'csv_file' in locals() and csv_file is not None:
            csv_file.close()
        for _, cap in cams:
            cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    live_localization()
