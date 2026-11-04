# 2026-10-29/30 舵机固件 V311 → V330 升级执行记录

- 依据：`docs/adr/ADR-004-舵机固件升级到V330.md`（45156a9，Thomas 2026-10-30 签字）
- 执行人：Thomas（全部上位机操作、臂上命令、断电重启、动臂）；审查方（流程指挥、参数文件比对）
- 结论：**7 颗舵机（J0–J6）全部升级到 V330；升级前后参数 25 项逐颗一致；整臂功能复验通过（Thomas 实机确认）**

## 1. 前置答复

### 1.1 厂家（FashionStar 厂家技术支持）

2026-10-29 第一轮（Thomas 转贴原文）：

> 1. 宇树的固件都是311版本的，不支持数据监控、停止、同步等指令。
> 2. 不需要拆机，可以通过我们上位机去进行升级……请确保升级到330版本即可使用上述功能。
> 3. 不升级也可以读取……但是建议还是用新版本会好一点。
> 4. 风险：单个舵机升级请务必不要断电，升级完毕之后再断电重启，然后升级另外一颗。如果升级过程中断电……会导致舵机变砖。
> 5. 保持锁力填0也是有锁力的。
> 6. 10ms是指在115200波特率下，任意指令和指令之间的间隔时间。115200下最长的一条指令大概3ms……也可以用到5ms，我们测试过是ok的。

另经 Thomas 转述（2026-10-29）：零点不会被清，建议升级后重置圈数；330 兼容 0x01/0x08/0x0A/0x11；参数可一键导出，**导入后 ID 会变为 0**。

2026-10-30 第二轮（微信截图：证据-05、证据-06）：都是一颗一颗升；参数不会变；没有时延要求；升级页选中哪颗就升哪颗；两款舵机用同一个 330 固件。

### 1.2 宇树

机身 Type-C 为出厂预留口，不对外开放，开发请走 RJ45（Thomas 2026-10-29 转述工单答复）。

## 2. 连接方法（B 方案：上位机经网口访问舵机总线）

```
Windows 上位机 COM5 ⇄ com0com 虚拟串口对 ⇄ COM12 ⇄ com2tcp ⇄ 网线 ⇄ 臂 <ARM_IP>:7777 ⇄ ~/bridge.py ⇄ /dev/ttyS4 ⇄ 舵机总线
```

| 环节 | 做法 | 踩过的坑 |
|---|---|---|
| Windows 网卡 | 静态 IP <WIN_IP>/24 | — |
| com0com | 用**签名版**驱动；`change CNCA0 PortName=COM5`，CNCB0 保持 COM12 | 未签名版报 0xE000022F；com0com 端口不在 Ports 类，上位机看不到 → 按 ReadMe FAQ 用 hdwwiz 添加一个「标准通信端口」COM5 并禁用，再把 CNCA0 改名为 COM5 |
| com2tcp | `.\com2tcp.exe --ignore-dsr \\.\COM12 <ARM_IP> 7777` | 不加 `--ignore-dsr` 会一直等 DSR，连不上 |
| 臂上 | 先停 4 个服务（ttyS4 只能一个进程用，F29），再 `python3 ~/bridge.py` | `/tmp` 断电即清空，脚本已放 `~/bridge.py`（2026-10-30） |
| 收尾 | 关上位机 → com2tcp `Ctrl-C` → bridge 自动退出 → 恢复 4 个服务，确认 4 行 active | 断电重启后服务会自启，需重新停 |

停 / 恢复服务（注意 `subscripber` 为原文拼写）：

```bash
sudo systemctl stop marm_controller marm_communication marm_control marm_subscripber; systemctl is-active marm_controller marm_communication marm_control marm_subscripber
sudo systemctl start marm_controller marm_communication marm_control marm_subscripber; systemctl is-active marm_controller marm_communication marm_control marm_subscripber
```

`~/bridge.py`（TCP ⇄ /dev/ttyS4 原样转发，单客户端）：

```python
# 只做 TCP <-> /dev/ttyS4 原样转发，单客户端，Ctrl-C 退出
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
```

## 3. 升级前基线（2026-10-29，只读）

上位机扫描：ID 0–6 七颗，型号均显示 515，序号均为 10000001，固件均为 V311（证据-01、证据-02）。电压 12.10–12.38 V，静止电流 0–1 mA，温度 26–29 ℃。

参数导出：`docs/hardware/Servo Motor/舵机参数 311固件/J0–J6_V311_20261029.usproj`。文件为 XML，每颗 25 项，**不含 ID 与波特率**；角度单位 0.1°（1800 = 180°），温度保护存原始码 651（界面显示 75 ℃）。

| 关节 | 功率保护 / 堵转功率上限 (mW) | Kp / Hold Kp | 死区 |
|---|---|---|---|
| J0 J1 J2 | 30000 | 850 | 3 |
| J3 J4 J5 | 20000 | 800 | 1 |
| J6 | 12000 | 800 | 1 |

其余 7 颗相同：低压 4000 / 高压 20000 mV、电流 8000 mA、角度限制开 ±180°、零点偏移 0、方向正转、Ki 0、Kd 50、Hold Kd 50、Bias 0、上电锁力关、缓启动关、指令响应 0。

## 4. 升级过程（2026-10-30）

升级工具：上位机 → 工具 → 舵机固件在线升级 → 无刷系列密钥登录 → 更新页 → 扫描 → 选中目标 ID → 开始更新（证据-04、证据-08）。可下载固件 V330。

| 顺序 | 关节 | 刷写 | 断电重启后扫描 | 参数比对 |
|---|---|---|---|---|
| 1 | J6 | 完成提示（证据-09） | 6 = V330，其余 V311，7 颗在、ID 不变（证据-10） | 一致 |
| 2 | J2 | 完成提示 | 2、6 = V330，其余 V311（证据-11） | 一致 |
| 3–7 | J5 J4 J3 J1 J0 | 完成 | 每颗升完均断电重启（Thomas 口述） | 一致 |
| 终 | 全部 | — | 0–6 全部 V330（证据-12） | — |

偏离：J2 刷写前未按流程截图确认选中项；重启后扫描证明升的确是 J2，无后果。J5 起 Thomas 自行连续执行，仍逐颗断电重启，符合 ADR-004。

参数比对方法：每颗升级后再导出 `Jn_V330_20261030.usproj`（`docs/hardware/Servo Motor/舵机参数 330 固件/`），去掉文件头随机 `Project ID` 后与 V311 文件 `diff`，7 颗输出均为空。未使用「导入参数」功能。

## 5. 收尾与复验

- 4 个服务恢复：`active` ×4（Thomas 2026-10-30 贴出输出）。
- 臂断电拆装后重新上电；Thomas 用原驱控方式实机确认：臂载服务正常，静止关节反馈正常，网页示教器运动功能正常，**关节功能均正常**（Thomas 2026-10-30 口述；无录像、无逐关节数值记录）。
- `journalctl -u marm_controller` 不加 sudo 只见 `Init`、`Exec` 两行（权限所限），未取得完整日志。

## 6. 遗留观察

- 升级后 J6、J2 静止电流读数 14 mA / 功率约 172 mW，未升级者 0–1 mA；上电锁力参数仍为 0。原因【未验证】（推测为 V330 读数口径变化），P0.4 实测时核对。
- 7 颗序号相同、型号均显示 515，二者均不能区分舵机或两款规格。
- V330 上 0x16 / 0x18 / 0x19 尚未实测（Q5 未关闭）。
