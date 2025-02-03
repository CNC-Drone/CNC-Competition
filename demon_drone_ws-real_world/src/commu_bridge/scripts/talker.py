#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import RCIn
import socket
import json
import time
import threading 
import struct
import os
import signal

target_pos = PoseStamped()
sendData = {'x':"0", 'y':"0", 'z':"0"}
remote_command = {'id':0, 'x':0, 'y':0}    # 速度值均代表百分比
clients = []  # 客户端列表
SWB = 0
SWD = 0

# drone odom callback
def drone_odom_callback(msg):
    sendData["x"] = msg.pose.pose.position.x
    sendData["y"] = msg.pose.pose.position.x
    sendData["z"] = msg.pose.pose.position.x

def rc_callback(msg):
    global SWB, SWD
    if msg.channels[7] < 1200:
        SWB = 0;
    elif msg.channels[7] < 1700:
        SWB = 1
    else:
        SWB = 2
    if msg.channels[5] < 1500:
        SWD = 0
    else:
        SWD = 1

# ROS相关
rospy.init_node("talker_node", anonymous=True)
# ros sub
rospy.Subscriber("/mavros/odometry/out", Odometry, drone_odom_callback)
rospy.Subscriber("/mavros/rc/in", RCIn, rc_callback)
# ros pub
keyborad_pub = rospy.Publisher("/keyboard/pose", PoseStamped, queue_size= 1)
send_flag = 0
recv_flag = 0

def sigint_handler(signalnum, handler):
    print("Ctrl+C Interrupt!")
    os._exit(0)
signal.signal(signal.SIGINT, sigint_handler)

# socket连接子线程
def HKServer(tcpClisock, addr):
    global remote_command, recv_flag
    rate = rospy.Rate(200)
    try:
        while not rospy.is_shutdown():         
            head = struct.unpack('<i', tcpClisock.recv(4))
            body = tcpClisock.recv(head[0])
            body = body.decode('UTF-8')
            remote_command = json.loads(body)
            print('|socket server| command ', remote_command)
            recv_flag = 1
            # send
            sendData_str = json.dumps(sendData)   # 将list转换为JSON字符串
            tcpClisock.send(struct.pack('<i', len(sendData_str)))
            tcpClisock.send(sendData_str.encode())

            rate.sleep()
    except Exception  as e:
        print("error:"), e
    finally: 
        print("客户端已关闭！"), addr
        clients.remove(tcpClisock)
        tcpClisock.close()

# socket线程
def socket_job(socket_server):
    global clients, isShutDownNode

    while not rospy.is_shutdown():
        print("waiting for connection...")
        tcpClisock, addr = socket_server.accept()    # 等待客户连接，服务器通过返回的新的socket对象(tcpClisock)进行与客户端通信
        # 启动新的进程与客户通信
        thread = threading.Thread(target=HKServer, args=(tcpClisock, addr),)
        thread.setDaemon(True)
        thread.start()
        # 记录新的客户
        clients.append(tcpClisock)
        print("现有客户端："), clients

def pub_job():
    global send_flag, SWB, SWD, recv_flag
    while True:
        if recv_flag == 1 and send_flag == 0 and SWB == 1 and SWD == 1:
            send_flag = 1
            recv_flag = 0
            target_pos.header.frame_id = "world";
            target_pos.pose.position.x = float(remote_command["x"])
            target_pos.pose.position.y = float(remote_command["y"])
            target_pos.pose.position.z = 1.5
            target_pos.pose.orientation.x = remote_command["id"]
            keyborad_pub.publish(target_pos)
            print("data had send!")
        if SWB != 1 or SWD != 1:
            send_flag = 0
        time.sleep(0.02)

# socket连接准备
host = '192.168.0.110'     # 在终端中输入netstat -an可以查看IP地址及端口
port = 60000
socket_server = socket.socket(socket.AF_INET,socket.SOCK_STREAM)  # 创建socket对象
socket_server.bind((host, port))   # 将socket绑定到服务器上，如果出错则引发socket.error异常
socket_server.listen(5)   # 准备套接字，数字代表最多连接数     
add_thread = threading.Thread(target = socket_job, args = (socket_server,))  # socket线程
add_thread.setDaemon(True)
add_thread.start()

pub_thread = threading.Thread(target = pub_job) 
pub_thread.setDaemon(True)
pub_thread.start()
rospy.spin()
socket_server.close()
print("服务器已关闭！")
