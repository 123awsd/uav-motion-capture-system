import json
import os
import numpy as np

def save_to_json(data, file_path):
    """
    将字典类型的数据保存为 JSON 文件。
    
    参数:
    - data: 要保存的数据（通常是 dict）
    - file_path: 保存路径，例如 "calibration_data.json"
    """
    try:
        with open(file_path, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=4)
        print(f"数据已成功保存到 {file_path}")
    except Exception as e:
        print(f"保存 JSON 文件出错: {e}")
def read_intrinsics_from_json(file_path):
    """
    从 JSON 文件中读取相机内参矩阵和畸变系数。
    
    参数：
        file_path (str): JSON 文件路径，包含相机内参矩阵。
        
    返回：
        dict: 包含相机内参矩阵的字典。
            - camera_matrix (np.ndarray): 相机内参矩阵
    """
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            data = json.load(f)

        camera_matrices = {}
        for camera_id, matrix in data.items():
            # 读取每个相机的内参矩阵
            camera_matrices[camera_id] = np.array(matrix)

        print("成功读取相机内参！")
        return camera_matrices
    
    except Exception as e:
        print(f"读取 JSON 文件时出错: {e}")
        return None
    
def read_transform_from_json(filename):
    """
    从 JSON 文件中读取旋转矩阵 R 和平移向量 t。

    参数:
        filename (str): JSON 文件路径
    
    返回:
        R (np.ndarray): 3x3 旋转矩阵
        t (np.ndarray): 3x1 平移向量
    """
    with open(filename, 'r') as f:
        data = json.load(f)
    
    R = np.array(data["R"], dtype=np.float32)
    t = np.array(data["t"], dtype=np.float32).reshape(3, 1)  # 保证列向量形式

    return R, t   

def read_extrinsics_from_json(file_path):
    """
    从 JSON 文件中读取多个相机的外参（R, t）。

    JSON 格式应为：
    {
        "camera_0": {
            "R": [[...], [...], [...]],
            "t": [...]
        },
        ...
    }

    返回：
        dict: 每个相机的外参字典，格式如下：
        {
            "camera_0": {
                "R": np.ndarray(3x3),
                "t": np.ndarray(3,)
            },
            ...
        }
    """
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            data = json.load(f)

        extrinsics = {}
        for camera_id, cam_data in data.items():
            R = np.array(cam_data["R"], dtype=np.float64)
            t = np.array(cam_data["t"], dtype=np.float64)
            extrinsics[camera_id] = {"R": R, "t": t}

        print("成功读取相机外参！")
        return extrinsics

    except Exception as e:
        print(f"读取相机外参时出错: {e}")
        return None
if __name__ == "__main__":   
    extrinsics = read_extrinsics_from_json("calibration_results/extrinsic_only.json")
    print(extrinsics)
    R,T=read_transform_from_json("calibration_results/transfrom.json")
    print(R)
    print(T)