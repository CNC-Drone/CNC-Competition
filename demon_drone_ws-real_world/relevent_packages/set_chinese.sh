# 安装依赖
sudo apt-get -y install language-pack-zh-* ttf-wqy-microhei language-pack-zh-han* ttf-wqy-zenhei fonts-droid-fallback
sudo apt-get -y install fonts-arphic-uming fonts-arphic-ukai

# 设置字体
sudo locale-gen "zh_CN.UTF-8"
export LC_ALL="zh_CN.utf8"
sudo update-locale LC_ALL="zh_CN.UTF-8" LANG="zh_CN.UTF-8" LC_MESSAGES=POSIX
sudo dpkg-reconfigure -f noninteractive locales