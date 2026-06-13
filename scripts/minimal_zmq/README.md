# minimal_zmq — 脱离 ROS 的 GELLO 遥操(ZMQ + franky）

不依赖 ROS2 / franka_ros2,不依赖 `gello` 包。GELLO 读数经 **ZMQ** 发给 follower,
真机控制用 **franky**(libfranka 的 Python 封装,Ruckig 在线轨迹)。自包含:`cd`
进本目录即可用。

录制 / 相机暂不做(留好接口),当前聚焦**控制链**,并按风险递增分三步验证。

## 架构

```
                          GELLO 右臂
                             │ 读数 + 标定
                             ▼
                        teleop.py ──ZMQ([q×7, grip])──▶  follower(REP server)
   （步骤1、3 同一个客户端，                              ├─ follower_sim.py : MuJoCo 可视化（步骤1）
     只换标定 sim/real）                                  └─ follower.py     : franky 真机（步骤2 只读 / 步骤3 控制）
```

关键对称性:
- `teleop.py` 在**步骤1 和步骤3 完全一样**,只是 `--calib` 不同、连的 follower 不同。
  所以步骤1(仿真)验证的就是步骤3(真机)要用的同一条控制客户端。
- `follower.py`(franky)在**步骤2 和步骤3 一样**,步骤2 加 `--read-only` 只读不动。

标定两套不能混用(见 `config.py` / `gello_driver.py`):
- `--calib sim` :modulo-wrap,用于步骤1(进 MuJoCo)
- `--calib real`:增量积分 + FR3 限位裁剪,用于步骤3(进 franky)
- 用错会把关节顶到限位(joint4 卡在 3.077 的老问题)。步骤脚本已自动传对。

## 步骤

### 0. 安装环境
```bash
bash scripts/minimal_zmq/00_setup.sh
```
建独立 `.venv`,装 numpy / pyzmq / mujoco / dm_control + 仓库内置 DynamixelSDK。
franky 不在这里装(仅 Linux+实时内核),到真机 PC 上单独 `pip install franky-control`。

### 1. 仿真遥操(无真机)
```bash
bash scripts/minimal_zmq/01_teleop_sim.sh              # 真 GELLO 驱动 MuJoCo 右臂
LEADER=dummy bash scripts/minimal_zmq/01_teleop_sim.sh # 无 GELLO,正弦兜底,纯验链路
```
GELLO →ZMQ→ MuJoCo 右臂动。验证读数 + sim 标定 + ZMQ 控制链。

Mac 可视化两种方式:
```bash
# A) 实时窗口(默认):弹 MuJoCo 窗口交互看(Mac 自动用 mjpython)
bash scripts/minimal_zmq/01_teleop_sim.sh

# B) 离屏录制成 MP4:无需窗口/mjpython,适合无显示器或录演示回放
RECORD=demo.mp4 SECS=10 LEADER=dummy bash scripts/minimal_zmq/01_teleop_sim.sh
```
端口被占时加 `PORT=17020`。

### 2. 真机状态可视化(只读,最安全)
```bash
bash scripts/minimal_zmq/02_visualize_robot.sh   # 单机:robot PC 带显示器
```
franky 只**读**真机状态 →ZMQ→ MuJoCo 镜像。**不发任何指令,机器人不会动。**
确认 franky 连得上、真机关节角和仿真对得上。
分机(Mac 看、robot PC 跑 franky):
```bash
# robot PC:
python scripts/minimal_zmq/follower.py --arm right --read-only --host 0.0.0.0
# Mac:
HOST=<robot_pc_ip> bash scripts/minimal_zmq/02_visualize_robot.sh
```

### 3. 真机遥操
```bash
bash scripts/minimal_zmq/03_teleop_robot.sh
RELDYN=0.05 bash scripts/minimal_zmq/03_teleop_robot.sh   # franky 更慢更稳
```
GELLO →ZMQ→ franky 真机。前两步通过后再跑。teleop 启动会先比较 GELLO 与真机当前
位姿,差太大直接中止(防止突然弹跳),否则缓动到位后开始跟随。

## 前置 / 配置

- 机器人 IP、ZMQ 端口、GELLO 串口、标定值都在 `config.py`。
  - 真机 IP:`ARMS[arm]["robot_ip"]`(右 192.168.20.12 / 左 192.168.20.11)
  - GELLO 串口:`PORTS`(macOS 名字会变,`ls /dev/cu.usbserial*` 后改)
- franky 仅 Linux + **PREEMPT_RT 实时内核**(`uname -a` 应含 `PREEMPT_RT`)。
- GELLO 舵机**全程不上电**,只读位置(没有手感/可能因重力下垂,但对遥操采集够用)。

## 文件
| 文件 | 作用 |
|---|---|
| `config.py` | 硬件/标定/网络常量(唯一需要改的配置) |
| `gello_driver.py` | GELLO Dynamixel 读数 + 两套标定 |
| `zmq_transport.py` | follower 协议的 ZMQ server/client + 后台线程 |
| `sim.py` | 自包含 MuJoCo 双臂 Panda 场景 |
| `follower_sim.py` | 步骤1:MuJoCo follower(ZMQ server) |
| `follower.py` | 步骤2/3:franky 真机 follower(ZMQ server) |
| `teleop.py` | 步骤1/3:GELLO→ZMQ 控制客户端 |
| `mirror.py` | 步骤2:真机状态→MuJoCo 镜像客户端 |

## 后续(留好空间,暂未做)
- 录制 / 相机节点:follower 的 `get_observations()` 已返回 q/dq/ee/grip,加相机
  ZMQ 节点 + 存盘客户端即可(不影响现有控制链)。
- 真机 torque/阻抗:franky 位置控制 + 内部柔顺已够;如需复刻 ROS2 那套 k/d 增益
  另说。
