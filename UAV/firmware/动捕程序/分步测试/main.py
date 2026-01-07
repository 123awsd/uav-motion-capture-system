import numpy as np
import cv2
from camera import points_capture ,camera_init
from localization import triangulate_points,triangulate_point,calculate_reprojection_errors
from mytool import save_to_json,read_intrinsics_from_json,read_extrinsics_from_json,read_transform_from_json
def system_init(camera_List =[0,2,4]):
    
    intrinsics = read_intrinsics_from_json("calibration_results/intrinsics_only.json")#读取内参
    print(intrinsics)
    extrinsics = read_extrinsics_from_json("calibration_results/extrinsic_only.json")#读取外参
    camera_poses = []
    for i in range(len(extrinsics)):
        camera_id = f"camera_{i}"
        if camera_id in extrinsics:
            pose = extrinsics[camera_id]
            camera_poses.append({
                "R": pose["R"],
                "t": pose["t"].reshape(3, 1)  # 可选：确保是列向量
            })
        else:
            print(f"警告: {camera_id} 不在外参数据中")
    print(camera_poses)

    #初始化相机
    cameras = camera_init(camera_List, 1280, 720)
    return cameras,intrinsics,extrinsics,camera_poses

if __name__ == "__main__":
    camera_List =[0,2,4]#ID
    cameras,intrinsics,extrinsics,camera_poses = system_init(camera_List)


    while True:
        show_images = []
        points_list = []

        for i in range(len(cameras)):
            cap = cameras[i]
            if cap is None:
                continue
            ret, imge = cap.read()
            if not ret or imge is None:
                print(f"摄像头 {i} 读取失败")
                continue

            points, show = points_capture(imge)
            show_images.append(show)
            points_list.append(points)

        #解算坐标
        # print(points_list)
        if all(len(points_list[i]) == 1 for i in range(len(camera_List))):
            # print(111)
            points = [list(point[0]) for point in points_list]
            # print(points)
            # print(222)
            object_point = triangulate_point(points, camera_poses,intrinsics)
            print(object_point)

            # ####################################坐标系转换
            # R,T=read_transform_from_json("calibration_results/transfrom.json")
            # R_inv = R.T  # 旋转矩阵的逆等于转置
            # t_inv = -np.dot(R_inv, T)  # 平移向量逆变换
            
            # # 转换坐标
            # camera_point_col = np.array(object_point).reshape(3, 1)
            # world_point = np.dot(R_inv, camera_point_col - T).flatten()
            # print(f"世界坐标系中的坐标:         {world_point}")
        # 显示图像
        for i, img in enumerate(show_images):
            window_width, window_height = 640, 360
            window_name = f"Camera {camera_List[i]}"
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)  # 允许调整大小
            cv2.imshow(window_name, img)
            cv2.resizeWindow(window_name, window_width, window_height)
            # cv2.imshow(f"Camera {i}", img)

        # 按下 q 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 释放资源
    for cap in cameras:
        if cap is not None:
            cap.release()
    cv2.destroyAllWindows()

    # 显示图像
    # window_width, window_height = 640, 360
    # for i, img in enumerate(all_images):
    #     if img is not None:
    #         window_name = f"Camera {camera_List[i]}"
    #         cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)  # 允许调整大小
    #         cv2.imshow(window_name, img)
    #         cv2.resizeWindow(window_name, window_width, window_height)
