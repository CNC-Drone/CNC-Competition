#!/bin/bash

fcu_url="/dev/ttyUSB0:921600"   # 飞控串口号


# 记录开始时间
start_time=$(date +%s)  

# 检测日志文件夹是否存在
script_path=$(readlink -f "$0")         # 当前脚本路径
parent_dir=$(dirname "$script_path")    # scripts位置
parent_dir1=$(dirname "$parent_dir")    # demon_drone_ws位置
LOG_DIR="$parent_dir1/output/debug_logs"
if [ ! -d "$LOG_DIR" ]; then
  mkdir -p "$LOG_DIR"
  echo "日志文件夹创建成功：$LOG_DIR"
else
  echo "日志文件夹已经存在：$LOG_DIR"
fi

source ~/.bashrc

echo "启动双目立体匹配中: roslaunch stereo stereo.launch"
STEREO_LOG_PATH="$parent_dir1/output/debug_logs/stereo.out"
nohup roslaunch stereo stereo.launch > $STEREO_LOG_PATH 2>&1 &
PID1=$!

# 检查Stereo是否启动成功
if [[ ! -f "$STEREO_LOG_PATH" ]]; then
    echo "stereo的日志文件不存在: $STEREO_LOG_PATH"
    return 1
fi

while ! grep -q "双目立体匹配线程开始" $STEREO_LOG_PATH; do
    echo "等待stereo启动中......"
    sleep 1
done
echo "stereo启动成功，日志文件将记录于：$STEREO_LOG_PATH"

MAVROS_LOG_PATH=$parent_dir1/output/debug_logs/px4.out
echo "正在启动MAVROS: roslaunch my_mavros px4.launch fcu_url:=$fcu_url  gcs_url:=udp-b://@"
nohup roslaunch mavros px4.launch fcu_url:=$fcu_url gcs_url:=udp-b://@ > $MAVROS_LOG_PATH 2>&1 &
PID2=$!

# 检查MAVROS是否启动成功
if [[ ! -f "$MAVROS_LOG_PATH" ]]; then
    echo "MAVROS的日志文件不存在: $MAVROS_LOG_PATH"
    return 1
fi

while ! grep -q $fcu_url $MAVROS_LOG_PATH; do
    echo "等待MAVROS启动中......"
    sleep 1
done
echo "MAVROS启动成功，日志文件将记录于：$MAVROS_LOG_PATH"


# # 检查mavros是否启动成功
# while true; do
#     # 获取 /mavros/state 话题的内容
#     state=$(rostopic echo -n 1 /mavros/state | grep "connected: True")

#     if [ -n "$state" ]; then
#         # rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0
#         # sleep 1
#         # rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0
#         # sleep 1
#         break
#     else
#         echo "正在等待mavros启动......"
#         sleep 1
#     fi
# done

VINS_LOG_PATH="$parent_dir1/output/debug_logs/vins.out"
nohup roslaunch vins stereo.launch > $VINS_LOG_PATH 2>&1 &
PID3=$!

# 检查vins是否启动成功
if [[ ! -f "$VINS_LOG_PATH" ]]; then
    echo "VINS的日志文件不存在: $VINS_LOG_PATH"
    return 1
fi

while ! grep -q "Initialization finish!" $VINS_LOG_PATH; do
    echo "等待VINS启动中......"
    sleep 1
done
echo "VINS启动成功，日志文件将记录于：$VINS_LOG_PATH"


# 生成close.sh脚本，写入kill命令
echo "#!/bin/bash" > $parent_dir1/scipts/close.sh
echo "kill $PID1 $PID2 $PID3 $PID4 $PID5 $PID6" >> $parent_dir1/scipts/close.sh

roslaunch commu_bridge transfer.launch