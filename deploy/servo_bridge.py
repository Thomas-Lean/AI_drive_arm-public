#!/usr/bin/env python3
"""舵机总线 TCP 转发（只在升级舵机固件时用，见 deploy/舵机升级.md）。

在臂载计算机上运行：把 TCP 7777 收到的字节原样写进 /dev/ttyS4，总线回的字节原样发回去，
供 Windows 上的厂家升级软件经网线访问舵机总线。单客户端；连接断开即退出，Ctrl-C 也可退出。
前提：臂上已停原厂 4 服务、本项目控制器也没在跑（/dev/ttyS4 同时只能一个进程用）；
臂上自带 python3 + pyserial。
用法（臂上）：python3 servo_bridge.py
"""
import socket, threading, serial
ser = serial.Serial('/dev/ttyS4', 115200, timeout=0.01)
srv = socket.socket(); srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(('0.0.0.0', 7777)); srv.listen(1)
print('等待连接 7777 ...', flush=True)
conn, addr = srv.accept(); print('已连接', addr, flush=True)
conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
def ser_to_net():
    while True:
        d = ser.read(256)
        if d: conn.sendall(d)
threading.Thread(target=ser_to_net, daemon=True).start()
try:
    while True:
        d = conn.recv(256)
        if not d: break
        ser.write(d)
finally:
    print('连接断开，退出', flush=True); conn.close(); ser.close()
