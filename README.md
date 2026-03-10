# WebRviz 本地可视化工具

## 项目简介

`WebRviz` 是一个运行在本地浏览器中的轻量级可视化工具，目标是将网页端显示与 `RViz` 的核心数据保持同步，并支持后续嵌入 Qt 界面。

实现功能：

- 在网页中显示机器人 URDF 模型。
- 同步 `TF`、`joint_states`、`PointCloud2`。
- 支持“一键同步 RViz”数据配置。
- 保持本地部署和低资源占用。

---

## 功能清单

- 支持通过 `/robot_description` 加载 URDF。
- 支持 URDF 地址（`URDF fallback URL`）。
- 支持 `package://` mesh 路径加载（通过本地 HTTP 挂载 ROS 包目录）。
- 同步订阅：
  - `/tf`
  - `/tf_static`
  - `/joint_states`
  - `sensor_msgs/PointCloud2`（自动发现 + 手动切换）
- 支持固定坐标系（`Fixed Frame`）显示。
- 支持 `Sync RViz` 按钮：从 RViz 配置提取关键信息并重建订阅。

---

## 环境要求

### 系统与中间件

- Ubuntu 20.04
- ROS Noetic
- MoveIt1

### 工具链

- Python 3.8 及以上
- Node.js 18 及以上（建议 Node.js 20 LTS）
- npm

### ROS 组件

- `rosbridge_server`
- `rosapi`

可用以下命令检查：

```bash
rosservice list | grep /rosapi
```

建议至少包含：

- `/rosapi/topics`
- `/rosapi/topic_type`
- `/rosapi/get_param`

---

## 目录结构

```text
web_rviz/
├─ src/
│  ├─ ros/
│  │  ├─ rosClient.ts
│  │  └─ pointcloud.ts
│  ├─ rviz/
│  │  └─ rvizConfig.ts
│  ├─ visualization/
│  │  ├─ sceneManager.ts
│  │  └─ robotLoader.ts
│  ├─ config.ts
│  ├─ main.ts
│  └─ style.css
├─ public/
│  ├─ rviz/moveit.rviz
│  └─ urdf/five_axis.urdf
├─ tools/
│  └─ serve_webrviz.py
├─ index.html
├─ package.json
└─ README.md
```

---

## 快速启动

### 1. 安装前端依赖

```bash
cd ~/webrviz/web_rviz
npm install
```

### 2. 启动 ROS 与 MoveIt

```bash
source ~/your_ws/devel/setup.bash
roslaunch your_moveit_config demo.launch
```

另开一个终端启动 rosbridge：

```bash
source ~/your_ws/devel/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

或者直接在`demo.launch`中启动`rosbrige node`和`rosapi node`
```xml
  <!-- Start rosbridge websocket server -->
  <node pkg="rosbridge_server"
        type="rosbridge_websocket"
        name="rosbridge_websocket"
        output="screen">
    <param name="port" value="9090"/>
  </node>
  
  <!-- rosapi -->
  <node pkg="rosapi"
        type="rosapi_node"
        name="rosapi"
        output="screen"/>
```
>注：建议将上面两个节点放在启动 Rviz 节点的后面

### 3. 构建前端资源

```bash
cd ~/webrviz/web_rviz
npm run build
```

### 4. 启动推荐静态服务（支持 package://）

```bash
python3 tools/serve_webrviz.py \
  --dist dist \
  --mount urdf_package_name=~/your_ws/src/urdf_package_name \
  --host 0.0.0.0 \
  --port 8080
```

### 5. 打开网页

浏览器访问：

```text
http://127.0.0.1:8080
```

---

## 页面配置说明

| 配置项 | 用途 | 默认值 |
|---|---|---|
| `rosbridge URL` | ROS WebSocket 地址 | `ws://<page-host>:9090` |
| `RViz config URL` | RViz 配置地址，用于一键同步 | `/rviz/moveit.rviz` |
| `URDF fallback URL` | `/robot_description` 不可用时使用 | `/urdf/five_axis.urdf` |
| `URDF package root URL` | `package://` 对应 HTTP 根路径 | `/ros_pkgs` |
| `PointCloud2 topic` | 点云话题，支持自动发现 | `/pointcloud/output` |

配置会保存到浏览器 `localStorage`：

```text
webrviz-runtime-config
```

---

## package:// 资源映射说明

### 背景

URDF 中常见 mesh 引用：

```xml
<mesh filename="package://urdf_package_name/meshes/base_link.STL"/>
```

浏览器无法直接读取本地文件系统路径（例如 `/home/...`），因此必须先映射为 HTTP URL。

### 本项目做法

通过命令参数：

```bash
--mount urdf_package_name=~/your_ws/src/urdf_package_name
```

将本地目录映射为：

- URL：`/ros_pkgs/urdf_package_name/...`
- 本地：`~/urdf_ws/src/urdf_package_name/...`

因此页面中应设置：

```text
URDF package root URL = /ros_pkgs
```

---

## Sync RViz 按钮机制

点击 `Sync RViz` 后执行：

1. 读取 `RViz config URL` 指向的配置文件。
2. 提取 `Fixed Frame` 和点云候选 topic。
3. 重新发现当前 ROS topics 与类型。
4. 选择可用点云话题并重建订阅。
5. 刷新网页端渲染状态。

>注：只同步数据，不会复制 RViz 的面板布局。

---

## Qt 界面集成

推荐用 `QWebEngineView` 打开本地网页地址：

```text
http://127.0.0.1:8080
```

建议将以下服务部署在同一主机：

- ROS 主节点
- rosbridge
- WebRviz 静态服务

可减少跨网络调试复杂度。

---

## 常见问题排查

### 1) 启动前端时报 `Unexpected token`

原因通常是 Node 版本过低。请升级到 Node 18+。

### 2) rosbridge 报 `/rosapi/topics_and_types does not exist`

当前版本已兼容该情况，使用 `/rosapi/topics` 和 `/rosapi/topic_type`。

### 3) 模型不显示且出现 `*.STL 404`

通常是 `package://` 资源未挂载。请检查：

- 是否使用 `tools/serve_webrviz.py` 启动。
- 是否配置 `--mount urdf_package_name=...`。
- 页面 `URDF package root URL` 是否为 `/ros_pkgs`。

### 4) 模型姿态与 RViz 不一致

请确认：

- `Fixed Frame` 与 RViz 一致。
- 点击 `Sync RViz` 后再观察。

### 5) Python 报 `type object is not subscriptable`

这是 Python 版本语法兼容问题（3.8 常见）。当前脚本已兼容 Python 3.8。

---

## 开发调试命令参考

```bash
rosnode list
rostopic list
rostopic hz /joint_states
rosparam get /robot_description | head
```

网页侧建议同时查看：

- 页面日志框（左侧）
- 浏览器开发者工具 Console