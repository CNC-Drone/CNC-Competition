#!/bin/sh

sudo apt install dos2unix

# 写入脚本内容
sudo touch /etc/systemd/system/virtual_screen.sh
sudo cat <<EOL > /etc/systemd/system/virtual_screen.sh
#!/bin/bash
sudo service gdm3 stop
sudo init 3
sudo /etc/NX/nxserver --restart
EOL

sudo dos2unix /etc/systemd/system/virtual_screen.sh # sudo apt install dos2unix
sudo chmod +x /etc/systemd/system/virtual_screen.sh
# 写入内容到 /etc/systemd/system/virtual_screen.service
sudo cat <<EOL > /etc/systemd/system/virtual_screen.service
[Unit]
After=sshd.service

[Service]
ExecStart=/etc/systemd/system/virtual_screen.sh

[Install]
WantedBy=default.target
EOL

sudo systemctl daemon-reload
sudo systemctl enable virtual_screen.service
sudo systemctl start virtual_screen.service
# sudo systemctl restart virtual_screen.service

# sudo systemctl status virtual_screen.service