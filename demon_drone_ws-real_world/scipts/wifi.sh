#!/bin/sh

nmcli device disconnect wlan0
nmcli connection delete MyHostspot

# #setup 2.4GHz
# nmcli connection add type wifi ifname wlan0 con-name MyHostspot autoconnect yes ssid Escort_Drone
# nmcli connection modify MyHostspot 802-11-wireless.mode ap 802-11-wireless.band bg ipv4.method shared 
# nmcli connection modify MyHostspot wifi-sec.key-mgmt wpa-psk
# nmcli connection modify MyHostspot wifi-sec.psk "1234qwer"
# nmcli connection modify MyHostspot ipv4.addresses 192.168.2.20/24
# nmcli connection modify MyHostspot ipv4.gateway 192.168.2.1
# nmcli connection up MyHostspot

#setup 5GHz
nmcli connection add type wifi ifname wlan0 con-name MyHostspot autoconnect yes ssid demon_1_5G
nmcli connection modify MyHostspot 802-11-wireless.mode ap 802-11-wireless.band a 802-11-wireless.channel 149 802-11-wireless.powersave 2 ipv4.method shared
nmcli connection modify MyHostspot wifi-sec.key-mgmt wpa-psk
nmcli connection modify MyHostspot wifi-sec.psk "1234qwer"
nmcli connection modify MyHostspot ipv4.addresses 192.168.88.20/24
nmcli connection modify MyHostspot ipv4.gateway 192.168.88.1
nmcli connection modify MyHostspot wifi.channel 149
nmcli connection up MyHostspot

sudo service gdm3 stop
sudo init 3
sudo /etc/NX/nxserver --restart


# chmod +X ~/bin/wifi.sh
# sudo cp ~/bin/wifi.service /etc/systemd/system/
# sudo systemctl daemon-reload
# sudo systemctl enable wifi.service
# systemctl status wifi.service 查看状态
# sudo systemctl start wifi.service

