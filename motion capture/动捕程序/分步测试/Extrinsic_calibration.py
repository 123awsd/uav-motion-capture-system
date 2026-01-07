
import os
import cv2
import json
import time
import numpy as np
from pathlib import Path
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R

from camera import points_capture, camera_init, DEV_TO_CAMID
from mytool import save_to_json

# ----------------------- I/O -----------------------

def load_intrinsics(intrinsics_json):
    with open(intrinsics_json, 'r', encoding='utf-8') as f:
        data = json.load(f)
    Ks = {}
    dists = {}
    for cam_id, v in data.items():
        if cam_id == "_meta":
            continue
        if "K" in v:
            Ks[cam_id] = np.array(v["K"], dtype=np.float64)
            dists[cam_id] = np.array(v.get("dist", [0,0,0,0,0]), dtype=np.float64).ravel()
        else:
            arr = np.array(v, dtype=np.float64)
            if arr.shape == (3,3):
                Ks[cam_id] = arr
                dists[cam_id] = np.zeros(5, dtype=np.float64)
    return Ks, dists

def dev_indices_to_cam_ids(dev_indices):
    return [DEV_TO_CAMID.get(idx, f"camera_{i}") for i, idx in enumerate(dev_indices)]

# ----------------------- Capture -----------------------

def capture_point_sets(camera_indices):
    cams = camera_init(camera_indices)  # [(dev_idx, cap), ...]
    if not cams:
        print("No cameras opened.")
        return []

    all_sets = []
    print("按 's' 保存当前一组（每台相机恰好检测到1个点），按 'q' 结束。")

    try:
        while True:
            per_cam_points = []
            per_cam_images = []

            for dev_idx, cap in cams:
                ok, frame = cap.read()
                if not ok or frame is None:
                    per_cam_points.append([])
                    per_cam_images.append(None)
                    continue
                pts, annotated = points_capture(frame)
                per_cam_points.append(pts)
                per_cam_images.append(annotated)

            # show
            for (dev_idx, _), img in zip(cams, per_cam_images):
                if img is None:
                    continue
                win = f"Camera /dev/video{dev_idx}"
                cv2.namedWindow(win, cv2.WINDOW_NORMAL)
                cv2.imshow(win, img)
                cv2.resizeWindow(win, 640, 360)

            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                break
            elif k == ord('s'):
                valid = True
                for (dev_idx,_), pts in zip(cams, per_cam_points):
                    if len(pts) != 1:
                        print(f"❌ /dev/video{dev_idx} 检测到 {len(pts)} 个点，需要恰好 1 个。")
                        valid = False
                if valid:
                    set_pts = [tuple(pts[0]) for pts in per_cam_points]
                    all_sets.append(set_pts)
                    print(f"✅ 保存第 {len(all_sets)} 组: {set_pts}")
    finally:
        for _, cap in cams:
            cap.release()
        cv2.destroyAllWindows()

    return np.array(all_sets, dtype=np.float64)  # [Nsets, Ncams, 2]

# ----------------------- Math helpers -----------------------

def decompose_essential(E):
    # E = U diag(1,1,0) Vt
    U, S, Vt = np.linalg.svd(E)
    if np.linalg.det(U) < 0: U[:, -1] *= -1
    if np.linalg.det(Vt) < 0: Vt[-1, :] *= -1
    W = np.array([[0, -1, 0],
                  [1,  0, 0],
                  [0,  0, 1]], dtype=np.float64)
    R1 = U @ W  @ Vt
    R2 = U @ W.T@ Vt
    t  = U[:, 2]
    # ensure rotations proper
    if np.linalg.det(R1) < 0: R1 *= -1
    if np.linalg.det(R2) < 0: R2 *= -1
    return [(R1,  t), (R1, -t), (R2, t), (R2, -t)]

def normalize_points(pts, K):
    Kinv = np.linalg.inv(K)
    pts_h = np.hstack([pts, np.ones((pts.shape[0],1))])
    norm = (Kinv @ pts_h.T).T
    return norm[:, :2] / norm[:, 2:3]

def projection_matrix(K, Rm, t):
    Rt = np.hstack([Rm, t.reshape(3,1)])
    return K @ Rt

def triangulate_dlt(point_list, P_list):
    # Build A from all cameras: for each cam, use x*P3 - P1 and y*P3 - P2
    A = []
    for (u, v), P in zip(point_list, P_list):
        p1 = P[0, :]; p2 = P[1, :]; p3 = P[2, :]
        A.append(u * p3 - p1)
        A.append(v * p3 - p2)
    A = np.array(A, dtype=np.float64)
    U, S, Vt = np.linalg.svd(A)
    Xh = Vt[-1, :]
    X = Xh[:3] / Xh[3]
    return X

def cheirality_count(R1, t1, R2, t2, K1, K2, pts1, pts2):
    # Using two cams, triangulate each correspondence, count X with Z>0 in both cameras
    P1 = projection_matrix(K1, R1, t1)
    P2 = projection_matrix(K2, R2, t2)
    cnt = 0
    for (p1, p2) in zip(pts1, pts2):
        X = triangulate_dlt([p1, p2], [P1, P2])
        Z1 = (R1 @ X + t1.reshape(3))[-1]
        Z2 = (R2 @ X + t2.reshape(3))[-1]
        if Z1 > 0 and Z2 > 0:
            cnt += 1
    return cnt

def accumulate_pose(R_prev, t_prev, R_rel, t_rel):
    R_abs = R_rel @ R_prev
    t_abs = R_rel @ t_prev + t_rel.reshape(3,1)
    return R_abs, t_abs

# ----------------------- Initialization -----------------------

def init_pairwise_poses(points_sets, Ks, cam_ids):
    """
    points_sets: [Nsets, Ncams, 2]
    Return list of absolute poses [{R,t}] length Ncams, with cam0 at origin.
    """
    Nsets, Ncams, _ = points_sets.shape
    poses = [dict(R=np.eye(3), t=np.zeros((3,1)))]
    # chain i -> i+1
    for i in range(Ncams-1):
        K1 = Ks[cam_ids[i]]
        K2 = Ks[cam_ids[i+1]]
        pts1 = points_sets[:, i, :].astype(np.float64)
        pts2 = points_sets[:, i+1, :].astype(np.float64)
        # Normalize
        x1n = normalize_points(pts1, K1)
        x2n = normalize_points(pts2, K2)
        # Essential via normalized 8-point with RANSAC on uncalibrated F then -> E
        F, _ = cv2.findFundamentalMat(pts1, pts2, method=cv2.FM_RANSAC, ransacReprojThreshold=1.0, confidence=0.999)
        if F is None or F.size == 0:
            raise RuntimeError("findFundamentalMat failed.")
        E = K2.T @ F @ K1
        # Decompose to 4 (R,t), choose via cheirality
        candidates = decompose_essential(E)
        best = None
        best_cnt = -1
        for (Rrel, trel) in candidates:
            cnt = cheirality_count(np.eye(3), np.zeros(3), Rrel, trel, K1, K2, pts1, pts2)
            if cnt > best_cnt:
                best_cnt = cnt
                best = (Rrel, trel)
        Rrel, trel = best
        R_prev = poses[-1]["R"]; t_prev = poses[-1]["t"]
        R_abs, t_abs = accumulate_pose(R_prev, t_prev, Rrel, trel)
        poses.append(dict(R=R_abs, t=t_abs))
    return poses

# ----------------------- Triangulation & error -----------------------

def triangulate_points_all(points_sets, poses, Ks, cam_ids):
    Nsets, Ncams, _ = points_sets.shape
    P_list = [projection_matrix(Ks[cid], poses[i]["R"], poses[i]["t"]) for i, cid in enumerate(cam_ids)]
    Xs = []
    for s in range(Nsets):
        pts = points_sets[s, :, :]
        X = triangulate_dlt(pts, P_list)
        Xs.append(X)
    return np.array(Xs, dtype=np.float64)  # [Nsets, 3]

def reproj_residual_vector(points_sets, Xs, poses, Ks, cam_ids):
    Nsets, Ncams, _ = points_sets.shape
    res = []
    for s in range(Nsets):
        X = Xs[s].reshape(3,1)
        for i, cid in enumerate(cam_ids):
            K = Ks[cid]
            Rm = poses[i]["R"]; t = poses[i]["t"]
            rvec = cv2.Rodrigues(Rm)[0]
            proj, _ = cv2.projectPoints(X.T, rvec, t, K, None)
            uv = proj.reshape(-1,2)[0]
            res.extend((uv - points_sets[s, i, :]).tolist())
    return np.array(res, dtype=np.float64)

# ----------------------- Bundle Adjustment -----------------------

def pack_params(poses, optimize_focal, Ks, cam_ids):
    params = []
    # camera 1..N: rvec(3), t(3), [optional focal scale s]
    for i in range(1, len(poses)):
        rvec = cv2.Rodrigues(poses[i]["R"])[0].reshape(3)
        t = poses[i]["t"].reshape(3)
        params.append(np.hstack([rvec, t]))
        if optimize_focal:
            params.append(np.array([1.0]))
    return np.hstack(params)

def unpack_params(x, n_cams, optimize_focal, Ks0, cam_ids):
    poses = [dict(R=np.eye(3), t=np.zeros((3,1)))]
    Klist = [Ks0[cam_ids[0]].copy()]
    idx = 0
    for i in range(1, n_cams):
        rvec = x[idx:idx+3]; idx += 3
        t = x[idx:idx+3].reshape(3,1); idx += 3
        Rm = cv2.Rodrigues(rvec)[0]
        poses.append(dict(R=Rm, t=t))
        if optimize_focal:
            s = float(x[idx]); idx += 1
            K = Ks0[cam_ids[i]].copy()
            K[0,0] *= s; K[1,1] *= s
            Klist.append(K)
        else:
            Klist.append(Ks0[cam_ids[i]])
    Ks_cur = {cid: Klist[i] for i, cid in enumerate(cam_ids)}
    return poses, Ks_cur

def bundle_adjust(points_sets, poses_init, Ks0, cam_ids, optimize_focal=False, verbose=1):
    n_cams = len(poses_init)
    x0 = pack_params(poses_init, optimize_focal, Ks0, cam_ids)

    def fun(x):
        poses, Ks_cur = unpack_params(x, n_cams, optimize_focal, Ks0, cam_ids)
        Xs = triangulate_points_all(points_sets, poses, Ks_cur, cam_ids)
        return reproj_residual_vector(points_sets, Xs, poses, Ks_cur, cam_ids)

    res = least_squares(fun, x0, method='trf', loss='cauchy', f_scale=2.0, xtol=1e-9, ftol=1e-9, max_nfev=2000, verbose=verbose)
    poses, Ks_cur = unpack_params(res.x, n_cams, optimize_focal, Ks0, cam_ids)
    return poses, Ks_cur, res

# ----------------------- Scale & world alignment -----------------------

def apply_metric_scale(Xs, poses, ref_pair=None, known_distance=None):
    if ref_pair is None or known_distance is None:
        return Xs, poses, 1.0
    i, j = ref_pair
    if i < 0 or j < 0 or i >= len(Xs) or j >= len(Xs):
        return Xs, poses, 1.0
    dist = np.linalg.norm(Xs[i] - Xs[j])
    if dist <= 1e-9:
        return Xs, poses, 1.0
    s = known_distance / dist
    # scale all translations and points (rotations unchanged)
    Xs_scaled = Xs * s
    poses_scaled = []
    for p in poses:
        poses_scaled.append(dict(R=p["R"].copy(), t=p["t"].copy()*s))
    return Xs_scaled, poses_scaled, s

# Placeholder for plane fit alignment
def align_world_Z_by_plane(Xs):
    # Fit plane ax+by+cz+d=0 via SVD; set normal -> (0,0,1)
    if len(Xs) < 3:
        return np.eye(3), np.zeros((3,1))
    P = Xs - np.mean(Xs, axis=0, keepdims=True)
    U,S,Vt = np.linalg.svd(P, full_matrices=False)
    normal = Vt[-1, :]
    normal = normal / (np.linalg.norm(normal) + 1e-9)
    # rotation from normal to [0,0,1]
    z = np.array([0,0,1.0])
    v = np.cross(normal, z)
    c = np.dot(normal, z)
    if np.linalg.norm(v) < 1e-9:
        Rw = np.eye(3)
    else:
        vx = np.array([[0, -v[2], v[1]],[v[2], 0, -v[0]],[-v[1], v[0], 0]])
        Rw = np.eye(3) + vx + vx@vx*((1-c)/(np.linalg.norm(v)**2))
    t = -Rw @ np.mean(Xs, axis=0, keepdims=True).T
    return Rw, t

# ----------------------- Main -----------------------

def solve_extrinsics_full(camera_indices=(0,2,4),
                          intrinsics_json="calibration_results/intrinsics_only.json",
                          save_path="calibration_results/extrinsic_only.json",
                          optimize_focal=False,
                          metric_pair=None, metric_distance=None,
                          do_alignment=False):
    Ks, dists = load_intrinsics(intrinsics_json)
    cam_ids = [DEV_TO_CAMID.get(idx, f"camera_{k}") for k, idx in enumerate(camera_indices)]
    for cid in cam_ids:
        if cid not in Ks:
            raise RuntimeError(f"缺少内参: {cid}")

    points_sets = capture_point_sets(camera_indices)  # [Nsets, Ncams, 2]
    if len(points_sets) < 8:
        print("⚠️ 建议至少 8 组以稳定初始化")

    # init pairwise via 4-solution + cheirality
    poses_init = init_pairwise_poses(points_sets, Ks, cam_ids)

    # BA
    poses_ba, Ks_cur, res = bundle_adjust(points_sets, poses_init, Ks, cam_ids, optimize_focal=optimize_focal, verbose=2)

    # Triangulate & error
    Xs = triangulate_points_all(points_sets, poses_ba, Ks_cur, cam_ids)
    residuals = reproj_residual_vector(points_sets, Xs, poses_ba, Ks_cur, cam_ids)
    px = np.abs(residuals).reshape(-1,2)
    mean_err = px.mean(axis=0).mean(); med_err = np.median(np.linalg.norm(px,axis=1))

    # Metric scale (optional)
    Xs_s, poses_s, scale = apply_metric_scale(Xs, poses_ba, metric_pair, metric_distance)

    # World alignment (optional)
    if do_alignment:
        Rw, tw = align_world_Z_by_plane(Xs_s)
        # transform poses (world->cam) by inserting world alignment on the right: new [R|t] = [R|t] @ [Rw|tw]
        new_poses = []
        for p in poses_s:
            Rn = p["R"] @ Rw
            tn = p["R"] @ tw + p["t"]
            new_poses.append(dict(R=Rn, t=tn))
        poses_s = new_poses

    # Save
    Path(os.path.dirname(save_path) or ".").mkdir(parents=True, exist_ok=True)
    out = {}
    for i, p in enumerate(poses_s):
        out[f"camera_{i}"] = {"R": p["R"].tolist(), "t": p["t"].reshape(3).tolist()}
    meta = {"optimize_focal": bool(optimize_focal), "reproj_mean_px": float(mean_err), "reproj_median_L2_px": float(med_err), "scale": float(scale)}
    out["_meta"] = meta
    save_to_json(out, save_path)
    print(f"[OK] 外参保存到 {save_path}")
    print(f"重投影误差：mean={mean_err:.3f}px, median(L2)={med_err:.3f}px")

if __name__ == "__main__":
    solve_extrinsics_full()
