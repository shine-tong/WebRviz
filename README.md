# WebRviz
[![CI](https://img.shields.io/badge/CI-GitHub_Actions-blue)](.github/workflows/ci.yml)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

## 项目简介

WebRviz 是一个面向本地 ROS 环境的轻量级浏览器可视化工具。当前分支已经切换到 ROS 2 Jazzy。

---

## 仓库内容

本仓库同时包含前端和 ROS2 侧辅助包：

- `web_rviz/`：浏览器前端，基于 Vite + TypeScript
- `webrviz_moveit_observer/`：ROS 2 辅助节点，用于转发 MoveIt 事件并提供接口文本查询
- `webrviz_interfaces/`：辅助节点使用的自定义 ROS 接口

---

## 功能特性

- 在浏览器中可视化 URDF、`/tf`、`/tf_static`、`/joint_states`、`sensor_msgs/msg/PointCloud2`
- 从 RViz 配置文件同步固定坐标系和点云显示候选
- 通过 `rosbridge + rosapi` 查看 ROS 话题、服务、参数和节点图谱
- 基于 `moveit_msgs/msg/DisplayTrajectory` 预览 MoveIt 规划轨迹
- 通过 `webrviz_moveit_observer` 显示 MoveIt 规划和执行事件
- 通过 `/webrviz/get_interface_text` 查看话题和服务的接口文本
- 打开轨迹曲线窗口查看关节位置、TCP 位置、速度、加速度和 effort

---

## 支持环境

- Ubuntu 24.04
- ROS 2 Jazzy
- MoveIt 2
- Python 3.10+
- Node.js 18+，推荐 Node.js 20 LTS
- `ros-jazzy-rosbridge-server`
- `ros-jazzy-rosapi`

推荐部署方式：

- Ubuntu 或 WSL2 运行 ROS 后端
- Ubuntu、WSL2 或 Windows 运行前端静态文件服务

---

## ROS 工作空间准备

先安装 ROS 侧依赖：

```bash
sudo apt update
sudo apt install ros-jazzy-rosbridge-server ros-jazzy-rosapi ros-jazzy-rosbridge-suite ros-jazzy-rosapi-library ros-jazzy-rosapi-msgs
```

将下面的 ROS2 辅助包放进工作空间：

```txt
webrviz_interfaces/
webrviz_moveit_observer/
```

构建本项目需要的 ROS 包：

```bash
cd ~/your_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select \
  webrviz_interfaces \
  webrviz_moveit_observer
source ~/your_ws/install/setup.bash
```

---

## 前端构建

在 `web_rviz/` 目录下安装依赖并构建：

```bash
cd ~/your_ws/src/WebRviz/web_rviz
npm install
npm run build
```

---

## 启动方式

### 一键启动方式

将 `scripts/demo_webrviz.launch.py` 启动脚本复制到 `your_moveit_config/launch` 文件夹下并修改 `moveit_config` 包名：

```python
...
    demo_launch = ExecuteProcess(
        cmd=[
            "/bin/bash",
            "-lc",
            (
                "set -o pipefail; "
                "stdbuf -oL -eL ros2 launch your_moveit_config demo.launch.py 2>&1 | "
                "stdbuf -oL -eL grep -E '^\\[(INFO|WARN|ERROR)\\] \\[launch\\]'"
            ),
        ],
        name="demo.launch.py",
        output="screen",
        emulate_tty=True,
    )
  ...
```

该 launch 会一次性启动：

- `demo.launch.py`
- `rosbridge_websocket`
- `rosapi_node`
- `webrviz_moveit_observer`

可选参数：

```bash
ros2 launch your_moveit_config demo_webrviz.launch.py \
  rosbridge_address:=0.0.0.0 \
  rosbridge_port:=9090 \
  rosapi_params_timeout:=15.0
```

说明：

- `rosbridge_websocket` 会继续输出到终端
- `rosapi` 和 `webrviz_moveit_observer` 默认写入 ROS 日志文件

### 分开启动方式

如果你要逐个组件调试，也可以继续分开启动：

```bash
ros2 launch your_moveit_config demo.launch.py
ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0 port:=9090
ros2 run rosapi rosapi_node --ros-args -p params_timeout:=15.0
ros2 launch webrviz_moveit_observer observer.launch.py
```

observer 会发布：

- `/webrviz/moveit/plan_event`
- `/webrviz/moveit/execute_event`

observer 还会提供：

- `/webrviz/get_interface_text`

---

## 静态文件服务

### Linux

```bash
cd ~/your_ws/src/WebRviz/web_rviz
python3 tools/serve_webrviz.py \
  --dist dist \
  --mount your_moveit_config=~/your_ws/src/WebRviz/your_moveit_config \
  --mount your_urdf=~/your_ws/src/WebRviz/your_urdf \
  --host 0.0.0.0 \
  --port 8080
```

### Windows

```powershell
cd C:\webrviz\web_rviz
py -3 .\tools\serve_webrviz.py `
  --dist .\dist `
  --mount "your_moveit_config=\path\your_moveit_config" `
  --mount "your_urdf=\path\your_urdf" `
  --host 0.0.0.0 `
  --port 8080
```

浏览器打开：

```text
http://127.0.0.1:8080
```

---

## 默认运行时配置

这些默认值来自 `web_rviz/src/config.ts`：

| 配置项 | 默认值 |
|---|---|
| `ROSBridge URL` | `ws://<page-host>:9090` |
| `RViz config URL` | `/ros_pkgs/your_moveit_config/config/moveit.rviz` |
| `URDF fallback URL` | `/ros_pkgs/your_urdf/urdf/your.urdf` |
| `URDF package root URL` | `/ros_pkgs` |
| `Fixed Frame` | `base_link` |
| `PointCloud2 topic` | `/pointcloud/output` |

说明：

- 前端会优先从 ROS 参数中读取 `robot_description`
- 如果参数读取失败，会回退到上面的 URDF 文件
- 浏览器端不会解析 `.xacro`
- 静态服务里的 mount 名称必须和 URDF 中的 `package://` 前缀一致

---

## 运行时数据来源

- 机器人模型：`robot_description` 或 URDF fallback
- 规划轨迹预览：`/display_planned_path`
- MoveIt 日志事件：`/webrviz/moveit/plan_event` 和 `/webrviz/moveit/execute_event`
- 话题和服务接口文本：`/webrviz/get_interface_text`
- ROS 元数据：`rosapi`

---

## 手工验证建议

所有组件启动后，建议按下面顺序验证：

1. 浏览器能够连接 `rosbridge`
2. WebRviz 日志中显示 `ROS 2 jazzy`
3. 机器人模型正常加载
4. `/tf`、`/tf_static`、`/joint_states` 正常更新
5. `/display_planned_path` 能驱动轨迹预览和轨迹曲线
6. 规划和执行时会产生 `/webrviz/moveit/plan_event` 与 `/webrviz/moveit/execute_event`
7. 话题和服务详情弹窗能够读取接口文本

常用检查命令：

```bash
ros2 service list | grep -E '/rosapi|/webrviz'
ros2 topic list | grep webrviz
ros2 topic echo /webrviz/moveit/plan_event --once
ros2 topic echo /webrviz/moveit/execute_event --once
```

---

## 常见问题

### 机器人模型不显示

- 先确认 ROS 参数里是否能读到 `robot_description`
- 如果参数不可用，确认 fallback 文件是否存在：
  `/ros_pkgs/your_urdf/urdf/your.urdf`
- 同时确认静态文件服务已经正确挂载：
  - `your_moveit_config`
  - `your_urdf`

### 参数加载很慢或部分失败

可以单独启动 rosapi 并增大超时：

```bash
ros2 run rosapi rosapi_node --ros-args -p params_timeout:=15.0
```

或者直接使用推荐的一键启动：

```bash
ros2 launch your_moveit_config demo_webrviz.launch.py
```

### 话题或服务详情为空

- 确认 `webrviz_moveit_observer` 已启动
- 确认 helper service 存在：

```bash
ros2 service list | grep /webrviz/get_interface_text
```

### 浏览器里没有 MoveIt 事件日志

检查 observer 话题是否存在：

```bash
ros2 topic list | grep /webrviz/moveit
```

正常情况下应看到：

- `/webrviz/moveit/plan_event`
- `/webrviz/moveit/execute_event`

---

## 开发模式

前端开发可直接运行：

```bash
cd ~/your_ws/src/WebRviz/web_rviz
npm run dev
```
