# 一、常见问题
## 1.1 nomachine黑屏
1）创建文件
~~~
sudo gedit /etc/systemd/system/demon_core.service
~~~
2）写入内容
~~~
[Unit]
After=sshd.service
 
[Service]
ExecStart=/home/khadas/bin/demon_core.sh
 
[Install]
WantedBy=default.target
~~~

3）demon_core.sh中的内容
~~~
<!-- sudo systemctl stop display-manager -->
sudo service gdm3 stop
sudo init 3
sudo /etc/NX/nxserver --restart
~~~

4）启动服务
~~~
sudo chmod 644 /etc/systemd/system/demon_core.service
sudo systemctl daemon-reload
sudo systemctl enable demon_core.service
systemctl list-unit-files | grep demon_core.service
~~~

## 1.2 编译内存不够
## 1.2.1 查看内存情况
~~~
free -h
~~~
## 1.2.2 创建swap文件
~~~
sudo fallocate -l 3G /swapfile
~~~
### 1.2.3 查看swap文件属性
~~~
ls -lh /swapfile
~~~
### 1.2.4 更改权限
~~~
sudo chmod 600 /swapfile
~~~
### 1.2.5 初始化为交换文件
~~~
sudo mkswap /swapfile
~~~
### 1.2.6 启用交换文件
~~~
sudo swapon /swapfile 
~~~
### 1.2.7 使其永久生效
~~~
sudo cp /etc/fstab /etc/fstab.bak
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
~~~

### 1.2.7 释放临时内存(可选)
~~~
swapoff-a 
~~~

## 1.3 运行程序报错：error while loading shared libraries: libnlopt.so.0
### 1.3.1 打开配置文件
~~~
sudo gedit /etc/ld.so.conf
~~~
### 1.3.2 修改配置
将/usr/local/lib加在尾部
### 1.3.3 保存/刷新
~~~
sudo ldconfig -v
~~~

# 二、安装依赖
## 2.1 ROS2安装
[官方教程](http://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
## 2.2 QGC安装
[github官网](https://github.com/mavlink/qgroundcontrol/releases)
[官网教程](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html)
~~~
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libqt5gui5 -y
sudo apt install libfuse2 -y
~~~

# 三、数据标定
~~~
rosbag record -O imu /mavros/imu/data
rosbag record -O stereo_imu  /mavros/imu/data /stereo/left/image_raw /stereo/right/image_raw
~~~
~~~
rosbag play -r 400 imu.bag
kalibr_calibrate_cameras --target checkerboard.yaml --bag stereo_imu.bag --models pinhole-radtan pinhole-radtan --topics /stereo/left/image_raw /stereo/right/image_raw
kalibr_calibrate_imu_camera --target checkerboard.yaml --cam camchain-stereo_imu.yaml --imu imu/imu.yaml --bag stereo_imu.bag
~~~

# 四、模型部署
https://blog.csdn.net/LateLinux/article/details/130150603

# 五、运行
## 5.1 ROS1

## 5.2 ROS2
~~~
echo "source ~/demon_drone_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
ros2 run demon_core demon_core_node 
~~~

