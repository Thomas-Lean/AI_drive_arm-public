# pico_probe · PICO 手柄输入链路（部署与探针）

PICO 4 Ultra 手柄（浏览器 WebXR）的位姿、扳机、握把经 TeleRobot 查看器进入 上位机 本机 ZMQ；
本目录的探针只读这路数据，**不连控制器、不发任何臂命令**。

```
PICO 浏览器（WebXR）
  │ HTTPS/WSS 8012（局域网）
  ▼
TeleRobot 查看器（上位机，Conda 环境 telerobot）
  │ ZMQ PUB tcp://127.0.0.1:55556，topic "xr"，60 Hz 发最新值（schema telerobot.xr.v1）
  ▼
pico_probe（上位机，只读）
```

## 1. 部署 TeleRobot（上位机，一次）

TeleRobot 用 Conda 环境 `telerobot`（Python 3.12）。Miniforge 装在用户目录，**不运行 `conda init`**
（不改 `~/.bashrc`，不碰系统 Python 与 ROS 2），用时 `source ~/miniforge3/bin/activate <环境名>` 激活。

```bash
# 1) Miniforge
curl -L -o ~/Miniforge3.sh "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash ~/Miniforge3.sh -b -p ~/miniforge3

# 2) TeleRobot 目录整份拷到 ~/TeleRobot（在开发机上执行）。
#    不要重新 clone：本地目录带上游库的适配补丁（帧接收时间戳、前台模式等）
rsync -a --exclude='__pycache__' --exclude='.pytest_cache' --exclude='runtime/' <TeleRobot 目录>/ <上位机>:~/TeleRobot/

# 3) 环境与依赖
cd ~/TeleRobot
~/miniforge3/bin/conda env create -f environment-linux.yml
source ~/miniforge3/bin/activate telerobot
python -m pip install --upgrade pip
python -m pip install -r requirements-linux.txt
python -m pip check                       # → No broken requirements found.
```

每次新开终端：

```bash
source ~/miniforge3/bin/activate telerobot && cd ~/TeleRobot
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"   # 用环境里的 libjpeg-turbo 3.x
```

预检与自带测试（不需要 PICO，不开相机）：

```bash
python start.py --check                   # → Configuration, imports and local API alignment: OK
python -m pip install pytest
python -m pytest tests -q                 # → 19 passed（含 ZMQ 收发、HTTPS/WSS 事件回环、断流 STALE）
```

## 2. 无 PICO 起服务（--no-image）

`config.yaml` 的 `xr.host` 先填 `127.0.0.1`（只本机可连）；接 PICO 前改成局域网地址（第 3 节）。

```bash
python start.py --no-image --ip 127.0.0.1
```

看到 `Ready [NO-IMAGE: synthetic frame]` 即起来了。另开终端确认 55556 在发布：

```bash
source ~/miniforge3/bin/activate telerobot
python - <<'PY'
import json, time, zmq
s = zmq.Context().socket(zmq.SUB)
s.setsockopt(zmq.SUBSCRIBE, b"xr"); s.setsockopt(zmq.RCVTIMEO, 2000)
s.connect("tcp://127.0.0.1:55556"); time.sleep(0.3)
t0, n = time.monotonic(), 0
while time.monotonic() - t0 < 3:
    _, payload = s.recv_multipart(); n += 1
m = json.loads(payload)
print(f"{n / 3:.0f} Hz", m["schema"], {k: v["fresh"] for k, v in m["samples"].items()})
PY
```

预期约 60 Hz、`telerobot.xr.v1`。没接 PICO 时 head/left/right 的 `fresh` 都是 False——
查看器不管有没有输入都按 60 Hz 发最新值，所以 ZMQ 帧率只说明查看器活着，不代表手柄数据在进来。
Ctrl+C 停服务。

## 3. 接 PICO

1. 上位机 与 PICO 连同一个 Wi-Fi；`ip -br addr` 查 `wlo1` 的地址（下称 `<IP>`）。
2. `config.yaml` 的 `xr.host` 改成 `<IP>`，然后 `python start.py --no-image --ip <IP>`（证书按新 IP 自动重签）。
3. PICO 浏览器开 `https://<IP>:8012/dashboard`，按系统提示信任本机证书；点「进入 PICO XR 视图 →」，
   在 Vuer 页面进入 XR 会话并允许追踪（dashboard 平面页本身不采集输入）。
4. 动手柄、按扳机/握把：dashboard 上的数值实时变化即输入链路通。

## 4. pico_probe（只读探针）与假数据

环境（一次；与 TeleRobot 的 `telerobot` 环境分开——cyclonedds 0.10.2 没有 Python 3.12 的包）：

```bash
~/miniforge3/bin/conda create -y -n d1t-teleop --override-channels -c conda-forge python=3.10 pip
source ~/miniforge3/bin/activate d1t-teleop
python -m pip install numpy==1.26.4 scipy==1.13.1 pyzmq==27.3.0 cyclonedds==0.10.2
```

`cyclonedds` 为导入 d1t_sdk（假 TCP 的 fk）所需，探针不建 DDS 参与者；TeleRobot 目录默认
`~/TeleRobot`，不在那里就设 `TELEROBOT_DIR`。

```bash
source ~/miniforge3/bin/activate d1t-teleop && cd ~/AI_drive_arm
# TeleRobot 查看器已起（第 2、3 节）时，另开一个终端：
python3 tools/pico_probe/pico_probe.py            # 右手柄；--side left 换左手
# 没有 PICO：假发布器画圆 + 握把/扳机按 12 s 时间表循环（先停掉查看器，两者共用 55556）
python3 tools/pico_probe/fake_xr_pub.py & python3 tools/pico_probe/pico_probe.py
```

状态行每秒一条：`接收频率 · LIVE/STALE · 离合 · 目标TCP（mm，另给相对接合点的位移 Δ）· j6 目标`。
离合=接合时握把 >0.85；脱开（<0.75）后目标保持不动；查看器重启（epoch 变化）后必须先松握把
才能再接合。假 TCP 是 fk(0,−40,60,0,30,0) 的结果——探针只算目标，不连控制器、不发臂命令。

离线测试（同环境；真实 ZMQ 回环，不需要查看器和 PICO）：

```bash
python3 tools/pico_probe/test_pico_probe.py        # 10 项：端点/接合不跳/方向/滞回/epoch/STALE/…
python3 tools/pico_probe/fake_xr_pub.py --restart-every 20 --stall-every 15 --stall-s 1.5   # 演示重启与断流
```

## 5. 排错

| 现象 | 处理 |
|---|---|
| PICO 打不开 `https://<IP>:8012` | 确认同网段、路由器没开 AP 隔离；本机有代理 TUN 网卡时，`ip route get <PICO 地址>` 应走 `wlo1`，被代理接管就把局域网段设为直连或先停代理 |
| 页面空白、查看器无任何日志 | 地址要写全 `https://`（只输 IP 会按 http 发，查看器直接断开）|
| 同网段能 ping 通但页面打不开，且本机另有一块网卡在同一网段（如接臂的有线网卡） | 回包走错网卡。`ip route get <PICO 地址>` 应显示无线网卡；不是就加 `sudo ip route add <PICO 地址>/32 dev <无线网卡> src <无线网卡地址>`（重启失效）|
| 握把一按就拖动浏览器窗口、探针一直 STALE | 还停在平面网页：在 Vuer 页面点 VR 入口进入沉浸模式 |
| `Port 8012 is unavailable` | 上一个实例没停：`ss -ltnp \| grep 8012` 查占用，停掉再起 |
| `start.py --check` 报 `No module named ...` | 没激活 `telerobot` 环境，或依赖不是在 `~/TeleRobot` 下装的 |
| dashboard 有画面但手柄 STALE | 没进 XR 会话或手柄没连上；dashboard 平面页不采集输入 |

TeleRobot 自身的完整说明（相机模式、显示模式、Web 入口）见 `~/TeleRobot/README.md`。
