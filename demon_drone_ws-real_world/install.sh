source $HOME/.bashrc

pip3 install ruamel.yaml loguru
sudo apt-get install -y v4l-utils qv4l2

script_dir=$(dirname "$(readlink -f "$0")")
parent_path=$(dirname "$script_dir")

version=$(grep VERSION_ID /etc/os-release | cut -d'=' -f2 | tr -d '"')
if [ "$version" == "20.04" ]; then
    sudo apt-get install -y ros-noetic-geodesy

    # 安装mavros
    sudo apt-get install -y ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-control-toolbox
    sudo cp -r $script_dir/relevent_packages/mavros-GeographicLib/GeographicLib/ /usr/share/
    sudo cp  $script_dir/relevent_packages/px4* /opt/ros/noetic/share/mavros/launch
elif [ "$version" == "18.04" ]; then
    sudo apt-get install -y ros-melodic-geodesy

    # 安装mavros
    sudo apt-get install -y ros-melodic-mavros ros-melodic-mavros-extras ros-melodic-control-toolbox
    sudo cp -r $script_dir/relevent_packages/mavros-GeographicLib/GeographicLib/ /usr/share/
    sudo cp  $script_dir/relevent_packages/px4* /opt/ros/melodic/share/mavros/launch
else
    echo "This script is designed to run on Ubuntu 18.04 or 20.04. Your version is $version."
fi


echo "进入文件夹：$script_dir"
cd $script_dir

catkin_make

# 加入bashrc
content_to_add="source $script_dir/devel/setup.bash"

# 检查是否已经包含了要添加的内容
if ! grep -qxF "$content_to_add" "$HOME/.bashrc"; then
    # 如果没有包含，则将内容添加到文件中
    echo "$content_to_add" >> "$HOME/.bashrc"
    echo "已将内容添加到 .bashrc 文件中"
else
    echo "内容已经存在于 .bashrc 文件中，无需重复添加"
fi

source $HOME/.bashrc
