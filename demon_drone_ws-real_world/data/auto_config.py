import numpy as np
import re
from ruamel.yaml import YAML    # pip3 install ruamel.yaml
from pathlib import Path
from loguru import logger       # pip3 install loguru
import os

if len(Path(__file__).parents) > 1:
    cali_data_path      = os.path.join(Path(__file__).parents[0], "stereo_imu-results-imucam.txt")
    vins_left_path      = os.path.join(Path(__file__).parents[1], "src/VINS-Fusion/config/left.yaml")
    vins_right_path     = os.path.join(Path(__file__).parents[1], "src/VINS-Fusion/config/right.yaml")
    vins_config_path    = os.path.join(Path(__file__).parents[1], "src/VINS-Fusion/config/stereo_imu_config.yaml")
    stereo_config_path  = os.path.join(Path(__file__).parents[1], "src/stereo/launch/stereo_config.yaml")
    transfer_path       = os.path.join(Path(__file__).parents[1], "src/commu_bridge/launch/transfer.yaml")
else:
    cali_data_path      = "./results-imucam-stereo_imu.txt"
    vins_left_path      = "../src/VINS-Fusion/config/left.yaml"
    vins_right_path     = "../src/VINS-Fusion/config/right.yaml"
    vins_config_path    = "../src/VINS-Fusion/config/stereo_imu_config.yaml"
    stereo_config_path  = "../src/stereo/launch/stereo_config.yaml"
    transfer_path       = "../src/commu_bridge/launch/transfer.yaml"

assert os.path.exists(cali_data_path), "请务必在data目录下放入标定文件：results-imucam-stereo_imu.txt"
logger.info(f"标定文件地址：{cali_data_path}")

# 读取标定文件
with open(cali_data_path, 'r') as file:
    lines = file.readlines()


T_ic_0 = None
T_ic_1 = None
timeshift = 0
Baseline = None     # 两个相机的外参矩阵
baseline_norm = 0   # 基线长度[m]
g_norm = 0         # [m/s^2]
Focal_length_0 = None               # 左焦距
Focal_length_1 = None               # 右焦距
Principal_point_0 = None            # 左中心
Principal_point_1 = None            # 右中心
Distortion_coefficients_0 = None    # 左畸变矩阵
Distortion_coefficients_1 = None    # 右畸变矩阵
acc_noise = None
gyr_noise = None
acc_w = None
gyr_w = None
for i, line in enumerate(lines):
    if 'T_ic:  (cam0 to imu0):' in line:
        T_ic_0 = np.array([list(map(float, line.strip().replace('[', '').replace(']', '').split())) for line in lines[i+1:i+5]])
        # logger.info(f"T_ic:  (cam0 to imu0):\n{T_ic_0}")

    if 'T_ic:  (cam1 to imu0):' in line:
        T_ic_1 = np.array([list(map(float, line.strip().replace('[', '').replace(']', '').split())) for line in lines[i+1:i+5]])
        # logger.info(f"T_ic:  (cam1 to imu0):\n{T_ic_1}")

    if 'timeshift cam0 to imu0:' in line:
        timeshift = float(lines[i+1])
        # logger.info(f"timeshift cam0 to imu0:{timeshift}")
        assert timeshift < -0.01, "timeshift cam0 to imu0 应该小于-0.01！"

    if 'Baseline (cam0 to cam1):' in line:
        Baseline = np.array([list(map(float, line.strip().replace('[', '').replace(']', '').split())) for line in lines[i+1:i+5]])
        # logger.info(f"Baseline:\n{Baseline}")
        # logger.info(abs(Baseline[0][3] + 0.06))
        assert Baseline[0][3] < 0, f"Baseline (cam0 to cam1)有误：{Baseline[0][3]}，转移矩阵x方向应该小于0！"

    if 'baseline norm:' in line:
        baseline_norm = float(line.split()[2])
        # logger.info(f"baseline_norm:{baseline_norm}")
        assert abs(baseline_norm - 0.06) < 0.001, f"基线有误：{baseline_norm}，请检查参数baseline_norm"

    if 'Gravity vector in target coords:' in line:
        g_norm = float(np.linalg.norm(np.array(list(map(float, lines[i+1].strip().replace('[', '').replace(']', '').split())))))
        # logger.info(f"g_norm:{g_norm}")

    if 'Focal length:' in line:
        if Focal_length_0 == None:
            Focal_length_0 = eval(re.search(r'Focal length: (\[.*?\])', line).group(1))
            # logger.info(f"Focal_length_0:\n{Focal_length_0}")
        else:
            Focal_length_1 = eval(re.search(r'Focal length: (\[.*?\])', line).group(1))
            # logger.info(f"Focal_length_1:\n{Focal_length_1}")

    if 'Principal point:' in line:
        if Principal_point_0 == None:
            Principal_point_0 = eval(re.search(r'Principal point: (\[.*?\])', line).group(1))
            # logger.info(f"Principal_point_0:\n{Principal_point_0}")
        else:
            Principal_point_1 = eval(re.search(r'Principal point: (\[.*?\])', line).group(1))
            # logger.info(f"Principal_point_1:\n{Principal_point_1}") 

    if 'Distortion coefficients:' in line:
        if Distortion_coefficients_0 == None:
            Distortion_coefficients_0 = eval(re.search(r'Distortion coefficients: (\[.*?\])', line).group(1))
            # logger.info(f"Distortion_coefficients_0:\n{Distortion_coefficients_0}")
        else:
            Distortion_coefficients_1 = eval(re.search(r'Distortion coefficients: (\[.*?\])', line).group(1))
            # logger.info(f"Distortion_coefficients_1:\n{Distortion_coefficients_1}") 
    
    if 'Noise density (discrete):' in line:
        if acc_noise == None:
            acc_noise = float(line.strip().split('Noise density (discrete): ')[1].strip())
            # logger.info(f"acc_noise:{acc_noise}") 
        else:
            gyr_noise = float(line.strip().split('Noise density (discrete): ')[1].strip())
            # logger.info(f"gyr_noise:{gyr_noise}") 
    
    if 'Random walk:' in line:
        if acc_w == None:
            acc_w = float(line.strip().split('Random walk: ')[1].strip())
            # logger.info(f"acc_w:{acc_w}") 
        else:
            gyr_w = float(line.strip().split('Random walk: ')[1].strip())
            # logger.info(f"gyr_w:{gyr_w}")    


# 创建一个YAML对象
yaml = YAML()
yaml.default_flow_style = None
yaml.version = "1.1"

head = "%YAML:1.0\n---\n"

# 修改left.yaml
with open(vins_left_path if os.path.exists(vins_left_path) else vins_left_path + ".example", 'r') as file:
    data = yaml.load(file)
    data['distortion_parameters']['k1'] = Distortion_coefficients_0[0]
    data['distortion_parameters']['k2'] = Distortion_coefficients_0[1]
    data['distortion_parameters']['p1'] = Distortion_coefficients_0[2]
    data['distortion_parameters']['p2'] = Distortion_coefficients_0[3]

    data['projection_parameters']['fx'] = Focal_length_0[0]
    data['projection_parameters']['fy'] = Focal_length_0[1]
    data['projection_parameters']['cx'] = Principal_point_0[0]
    data['projection_parameters']['cy'] = Principal_point_0[1]

with open(vins_left_path, 'w') as file:
    yaml.dump(data, file)
with open(vins_left_path, 'r') as file:
    txt = file.read()
    with open(vins_left_path, 'w') as file:
        file.write(head + txt)

# 修改right.yaml
with open(vins_right_path if os.path.exists(vins_right_path) else vins_right_path + ".example", 'r') as file:
    data = yaml.load(file)
    data['distortion_parameters']['k1'] = Distortion_coefficients_1[0]
    data['distortion_parameters']['k2'] = Distortion_coefficients_1[1]
    data['distortion_parameters']['p1'] = Distortion_coefficients_1[2]
    data['distortion_parameters']['p2'] = Distortion_coefficients_1[3]

    data['projection_parameters']['fx'] = Focal_length_1[0]
    data['projection_parameters']['fy'] = Focal_length_1[1]
    data['projection_parameters']['cx'] = Principal_point_1[0]
    data['projection_parameters']['cy'] = Principal_point_1[1]

with open(vins_right_path, 'w') as file:
    yaml.dump(data, file)
with open(vins_right_path, 'r') as file:
    txt = file.read()
    with open(vins_right_path, 'w') as file:
        file.write(head + txt)

# 修改stereo_config.yaml
with open(stereo_config_path if os.path.exists(stereo_config_path) else stereo_config_path + ".example", 'r') as file:
    data = yaml.load(file)
    data['distortion_coeffs1']['data'] = Distortion_coefficients_0 + [0]
    data['distortion_coeffs2']['data'] = Distortion_coefficients_1 + [0]

    data['camera_matrix1']['data'][0] = Focal_length_0[0]
    data['camera_matrix1']['data'][4] = Focal_length_0[1]
    data['camera_matrix1']['data'][2] = Principal_point_0[0]
    data['camera_matrix1']['data'][5] = Principal_point_0[1]

    data['camera_matrix2']['data'][0] = Focal_length_1[0]
    data['camera_matrix2']['data'][4] = Focal_length_1[1]
    data['camera_matrix2']['data'][2] = Principal_point_1[0]
    data['camera_matrix2']['data'][5] = Principal_point_1[1]   

    data['rotation_matrix']['data'] = Baseline[:3, :3].flatten().tolist()
    data['translation_coeffs']['data'] = Baseline[:3, -1].flatten().tolist()

    data['imu_T_cam0']['data'] = T_ic_0.flatten().tolist()

with open(stereo_config_path, 'w') as file:
    yaml.dump(data, file)
with open(stereo_config_path, 'r') as file:
    txt = file.read()
    with open(stereo_config_path, 'w') as file:
        file.write(head + txt)

# 修改transfer.yaml
with open(transfer_path if os.path.exists(transfer_path) else transfer_path + ".example", 'r') as file:
    data = yaml.load(file)
    data['imu_T_cam0']['data'] = T_ic_0.flatten().tolist()

with open(transfer_path, 'w') as file:
    yaml.dump(data, file)
with open(transfer_path, 'r') as file:
    txt = file.read()
    with open(transfer_path, 'w') as file:
        file.write(head + txt)

# 修改stereo_imu_config.yaml
with open(vins_config_path if os.path.exists(vins_config_path) else vins_config_path + ".example", 'r') as file:
    data = yaml.load(file)
    data['body_T_cam0']['data'] = T_ic_0.flatten().tolist()
    data['body_T_cam1']['data'] = T_ic_1.flatten().tolist()
    data['acc_n'] = acc_noise
    data['gyr_n'] = gyr_noise
    data['acc_w'] = acc_w
    data['gyr_w'] = gyr_w
    data['g_norm'] = g_norm
    data['td'] = timeshift

with open(vins_config_path, 'w') as file:
    yaml.dump(data, file)
with open(vins_config_path, 'r') as file:
    txt = file.read()
    with open(vins_config_path, 'w') as file:
        file.write(head + txt)

logger.info("完成配置文件修改")