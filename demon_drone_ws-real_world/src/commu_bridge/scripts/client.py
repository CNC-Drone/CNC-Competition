#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket
import time
import json
import struct
# 创建一个tcp/ip协议的套接字
clientSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# 创建一个udp/ip的套接字
host = "192.168.0.110"
port = 60000
# 开始连接服务器地址
clientSocket.connect((host, port))

sendData = {'id':0, 'x':3.121313, 'y':1.12313}
if clientSocket is None:
    print("无法连接当前的服务器！")
else:
    print("已经连接服务器---> oK")
    try:
        while True:
            sendData_str = json.dumps(sendData)   # 将list转换为JSON字符串
            clientSocket.send(struct.pack('<i', len(sendData_str)))
            clientSocket.send(sendData_str.encode())
            # head = struct.unpack('<i', clientSocket.recv(4))
            # body = clientSocket.recv(head[0])
            # body = body.decode('UTF-8')
            # remote_command = json.loads(body)
            # print('|socket server| command ', remote_command)
            time.sleep(0.5)
    except socket.error:
        print("error:", socket.error)
    finally:
        clientSocket.close()
        print("服务器关闭")