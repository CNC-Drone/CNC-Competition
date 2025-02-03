import numpy as np
import re
from ruamel.yaml import YAML    # pip3 install ruamel.yaml
# import yaml

# 打开文件
with open('results-imucam-stereo_imu.txt', 'r') as file:
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
        print(f"T_ic_0:{T_ic_0}\n")

    if 'T_ic:  (cam1 to imu0):' in line:
        T_ic_1 = np.array([list(map(float, line.strip().replace('[', '').replace(']', '').split())) for line in lines[i+1:i+5]])
        print(f"T_ic_1:{T_ic_1}\n")

    if 'timeshift cam0 to imu0:' in line:
        timeshift = float(lines[i+1])
        print(f"timeshift:{timeshift}\n")

    if 'Baseline (cam0 to cam1):' in line:
        Baseline = np.array([list(map(float, line.strip().replace('[', '').replace(']', '').split())) for line in lines[i+1:i+5]])
        print(f"Baseline:{Baseline}\n")

    if 'baseline norm:' in line:
        baseline_norm = float(line.split()[2])
        print(f"baseline_norm:{baseline_norm}\n")

    if 'Gravity vector in target coords:' in line:
        g_norm = float(np.linalg.norm(np.array(list(map(float, lines[i+1].strip().replace('[', '').replace(']', '').split())))))
        print(f"g_norm:{g_norm}\n")

    if 'Focal length:' in line:
        if Focal_length_0 == None:
            Focal_length_0 = eval(re.search(r'Focal length: (\[.*?\])', line).group(1))
            print(f"Focal_length_0:{Focal_length_0}\n")
        else:
            Focal_length_1 = eval(re.search(r'Focal length: (\[.*?\])', line).group(1))
            print(f"Focal_length_1:{Focal_length_1}\n")

    if 'Principal point:' in line:
        if Principal_point_0 == None:
            Principal_point_0 = eval(re.search(r'Principal point: (\[.*?\])', line).group(1))
            print(f"Principal_point_0:{Principal_point_0}\n")
        else:
            Principal_point_1 = eval(re.search(r'Principal point: (\[.*?\])', line).group(1))
            print(f"Principal_point_1:{Principal_point_1}\n") 

    if 'Distortion coefficients:' in line:
        if Distortion_coefficients_0 == None:
            Distortion_coefficients_0 = eval(re.search(r'Distortion coefficients: (\[.*?\])', line).group(1))
            print(f"Distortion_coefficients_0:{Distortion_coefficients_0}\n")
        else:
            Distortion_coefficients_1 = eval(re.search(r'Distortion coefficients: (\[.*?\])', line).group(1))
            print(f"Distortion_coefficients_1:{Distortion_coefficients_1}\n") 
    
    if 'Noise density (discrete):' in line:
        if acc_noise == None:
            acc_noise = float(line.strip().split('Noise density (discrete): ')[1].strip())
            print(f"acc_noise:{acc_noise}\n") 
        else:
            gyr_noise = float(line.strip().split('Noise density (discrete): ')[1].strip())
            print(f"gyr_noise:{gyr_noise}\n") 
    
    if 'Random walk:' in line:
        if acc_w == None:
            acc_w = float(line.strip().split('Random walk: ')[1].strip())
            print(f"acc_w:{acc_w}\n") 
        else:
            gyr_w = float(line.strip().split('Random walk: ')[1].strip())
            print(f"gyr_w:{gyr_w}\n")    


# 创建一个YAML对象
yaml = YAML()
yaml.default_flow_style = None
yaml.version = "1.1"

head = """%YAML:1.0
---
"""

# 修改left.yaml
with open('left.yaml', 'r') as file:
    data = yaml.load(file)
    data['distortion_parameters']['k1'] = Distortion_coefficients_0[0]
    data['distortion_parameters']['k2'] = Distortion_coefficients_0[1]
    data['distortion_parameters']['p1'] = Distortion_coefficients_0[2]
    data['distortion_parameters']['p2'] = Distortion_coefficients_0[3]

    data['projection_parameters']['fx'] = Focal_length_0[0]
    data['projection_parameters']['fy'] = Focal_length_0[1]
    data['projection_parameters']['cx'] = Principal_point_0[0]
    data['projection_parameters']['cy'] = Principal_point_0[1]

with open('left.yaml', 'w') as file:
    yaml.dump(data, file)
with open('left.yaml', 'r') as file:
    txt = file.read()
    with open('left.yaml', 'w') as file:
        file.write(head + txt)

# 修改right.yaml
with open('right.yaml', 'r') as file:
    data = yaml.load(file)
    data['distortion_parameters']['k1'] = Distortion_coefficients_1[0]
    data['distortion_parameters']['k2'] = Distortion_coefficients_1[1]
    data['distortion_parameters']['p1'] = Distortion_coefficients_1[2]
    data['distortion_parameters']['p2'] = Distortion_coefficients_1[3]

    data['projection_parameters']['fx'] = Focal_length_1[0]
    data['projection_parameters']['fy'] = Focal_length_1[1]
    data['projection_parameters']['cx'] = Principal_point_1[0]
    data['projection_parameters']['cy'] = Principal_point_1[1]

with open('right.yaml', 'w') as file:
    yaml.dump(data, file)
with open('right.yaml', 'r') as file:
    txt = file.read()
    with open('right.yaml', 'w') as file:
        file.write(head + txt)

# 修改stereo_config.yaml
with open('stereo_config.yaml', 'r') as file:
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
with open('stereo_config.yaml', 'w') as file:
    yaml.dump(data, file)
with open('stereo_config.yaml', 'r') as file:
    txt = file.read()
    with open('stereo_config.yaml', 'w') as file:
        file.write(head + txt)

# 修改stereo_imu_config.yaml
with open('stereo_imu_config.yaml', 'r') as file:
    data = yaml.load(file)
    data['body_T_cam0']['data'] = T_ic_0.flatten().tolist()
    data['body_T_cam1']['data'] = T_ic_1.flatten().tolist()
    data['acc_n'] = acc_noise
    data['gyr_n'] = gyr_noise
    data['acc_w'] = acc_w
    data['gyr_w'] = gyr_w
    data['g_norm'] = g_norm
    data['td'] = timeshift

with open('stereo_imu_config.yaml', 'w') as file:
    yaml.dump(data, file)
with open('stereo_imu_config.yaml', 'r') as file:
    txt = file.read()
    with open('stereo_imu_config.yaml', 'w') as file:
        file.write(head + txt)