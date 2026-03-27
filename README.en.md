# WebRviz Local Visualization Tool
[![CI](https://img.shields.io/badge/CI-passing-green)](.github/workflows/ci.yml)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![ZH](https://img.shields.io/badge/README-ZH-D4380D)](README.md)

## Overview

`WebRviz` is a lightweight visualization tool that runs in a local browser. Its goal is to keep the web view synchronized with the core data shown in `RViz`, while remaining easy to embed into a Qt-based desktop UI later.

For ROS2 Jazzy, please use the [jazzy branch](https://github.com/shine-tong/WebRviz/tree/jazzy).

---

## Features

- Displays robot URDF models in the browser and synchronizes `/tf`, `/tf_static`, `/joint_states`, and `sensor_msgs/PointCloud2`.
- Supports one-click RViz synchronization, including the fixed frame and candidate point cloud topics.
- The left panel supports Chinese/English switching, light/dark theme switching, and adaptive layout.
- The right `Robot State` panel includes joint angles, Cartesian position, and a TF tree. The TF tree shows the `link -> joint -> link` hierarchy and supports clicking a link or joint to inspect details.
- The right `ROS Info` panel lets you browse topics, services, and params, and inspect message, service, and parameter details.
- The central 3D view supports showing all TF axes, hiding all TF axes, or showing only selected Link TF axes, and includes a reset-view button.
- Supports MoveIt planned trajectory preview, including the trajectory line and TCP start/end pose markers.
- Supports a `ROS Graph` dialog that uses `rosapi` `nodes` / `node_details` snapshots to display a detailed `Nodes/Topics (all)` graph and a node communication graph.
- Supports a `Motion Charts` dialog that reads `trajectory_msgs/JointTrajectoryPoint[]` from `/move_group/result` and plots joint position, TCP position, velocity, acceleration, and effort curves with unit switching and hover readouts.
- Trajectory recording and playback support pause, progress dragging, time display, and playback preview.
- The log panel in the lower-left corner records connection, synchronization, and trajectory operations, and separately reports MoveIt planning and execution status.
- Runs locally with relatively low resource usage, making it suitable for desktop embedding or LAN deployment.

![WebRviz](web_rviz/assets/webrviz.png)

---

## Requirements

### System and Middleware

- Ubuntu 20.04
- ROS Noetic
- MoveIt1

### Toolchain

- Python 3.8+
- Node.js 18+ (Node.js 20 LTS recommended)
- npm

### ROS Components

- `rosbridge_server`
- `rosapi`
- `rospkg`

You can verify them with:

```bash
rosservice list | grep /rosapi
```

At minimum, these services should be available:

- `/rosapi/topics`
- `/rosapi/topic_type`
- `/rosapi/get_param`
- `/rosapi/nodes`
- `/rosapi/node_details`

---

## Quick Start

### 1. Build from Source

#### 1.1 Clone the Repository and Install Frontend Dependencies

```bash
git clone https://github.com/shine-tong/WebRviz.git
cd ~/WebRviz/web_rviz
npm install
```

#### 1.2 Start ROS and MoveIt

```bash
source ~/your_ws/devel/setup.bash
roslaunch your_moveit_config demo.launch
```

If you want `package://` resources to be exposed automatically over HTTP for WebRviz, make sure the `webrviz_asset_server` package and the robot package that owns `demo.launch` are in the same ROS workspace and have already been built.

Start rosbridge in another terminal:

```bash
source ~/your_ws/devel/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

You can also start `rosbridge`, `rosapi`, and the WebRviz asset server directly inside `demo.launch`:

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

<!-- WebRviz package:// asset server -->
<include file="$(find webrviz_asset_server)/launch/asset_server.launch">
  <arg name="host" value="0.0.0.0"/>
  <arg name="port" value="8081"/>
</include>
```

> Note: It is recommended to start the nodes above after the RViz node. With the default setup, make sure both ports `9090` and `8081` are reachable.

#### 1.3 Build Frontend Assets

```bash
cd ~/WebRviz/web_rviz
npm run build
```

#### 1.4 Start the Static Server

```bash
python3 tools/serve_webrviz.py \
  --dist dist \
  --host 0.0.0.0 \
  --port 8080
```

By default, after the frontend connects to rosbridge it will automatically resolve `package://` assets to:

```text
http://<rosbridge-host>:8081/ros_pkgs/<package_name>/...
```

If your `demo.launch` does not start `webrviz_asset_server`, you can still use `--mount urdf_package_name=...` as a fallback.

#### 1.5 Open the Web Page

```text
http://127.0.0.1:8080
```

### 2. Use the Release Package

#### 2.1 Extract the Downloaded Tarball

```bash
tar -xzf webrviz-<version>.tar.gz
cd webrviz
```

#### 2.2 Start the Static Server

```bash
./start.sh
```

The default port is `8080`. You can also configure it with environment variables:

```bash
export WEBRVIZ_HOST=0.0.0.0 \
WEBRVIZ_PORT=8080 \
WEBRVIZ_MOUNTS="--mount urdf_package_name=~/your_ws/src/urdf_package_name"
```

> Note: `WEBRVIZ_MOUNTS` is only a fallback when `webrviz_asset_server` is not enabled. Its value is passed directly to `tools/serve_webrviz.py`.

#### 2.3 Open the Web Page

```text
http://127.0.0.1:8080
```

### 3. Run on Windows

#### 3.1 Installation

> On Windows, follow the installation steps above.

#### 3.2 Pre-Launch Checklist

- Make sure `demo.launch`, `rosbridge`, and `webrviz_asset_server` are already running.
- If `ufw` is enabled, open the firewall ports:
  ```bash
  sudo ufw allow 9090/tcp
  sudo ufw allow 8081/tcp
  ```
- Check the Ubuntu IP address:
  ```bash
  hostname -I
  ```

#### 3.3 Start the Frontend

```bash
python tools/serve_webrviz.py \
  --dist dist \
  --host 0.0.0.0 \
  --port 8080
```

After the server starts, open the page and enter the Ubuntu IP in the `rosbridge URL` field:

```text
ws://yourUbuntu-ip:9090
```

If the connection fails, check:

- Whether Windows and Ubuntu are on the same network and can reach each other.
- Whether Ubuntu is listening on port `9090`:
  ```bash
  ss -lntp | grep 9090
  ```
- Whether Ubuntu is listening on port `8081`:
  ```bash
  ss -lntp | grep 8081
  ```
- Whether the URL prefix is `ws://` rather than `http://`.

---

## Development and Preview

```bash
# Development mode
npm run dev

# Preview the production build
npm run preview
```

---

## Page Configuration

| Setting | Purpose | Default |
|---|---|---|
| `rosbridge URL` | ROS WebSocket endpoint | `ws://<page-host>:9090` |
| `RViz config URL` | RViz config file used by one-click sync | `/rviz/moveit.rviz` |
| `URDF fallback URL` | Used when `/robot_description` is unavailable | `/urdf/five_axis.urdf` |
| `URDF package root URL` | HTTP root used to resolve `package://` | `/ros_pkgs` |
| `PointCloud2 topic` | Point cloud topic, supports auto-discovery | `/pointcloud/output` |

Configuration and UI preferences are stored in browser `localStorage`:

```text
webrviz-runtime-config
webrviz-language
webrviz-theme
```

## Right Sidebar

> Hidden by default. Use the middle arrow button to expand or collapse it.

### 1. Robot State

- Joint Angles: shows the current `/joint_states`, with `deg / rad` unit switching.
- Cartesian Position: lets you choose the `TCP link`, and provides the same length and angle unit switches used by the motion analysis view.
- TF Tree: always shows the current TF hierarchy in `link -> joint -> link` order. Clicking a link or joint opens its detail dialog.
- `Show All / Hide All`: controls whether TF axes are shown in the 3D view.
- `Select Link`: lets you choose one or more links and show only those TF axes in the 3D view.

### 2. ROS Info

- The `ROS Info` panel contains `Topics`, `Services`, and `Params`, and `Topics/Services` both show the corresponding message/service types.
- Clicking a topic shows its message structure details.
- Clicking a service shows its service definition, including Request/Response.
- Clicking a parameter shows its current value; you can also load all parameter values with one click.

## Center 3D View

- The top-left <img src="web_rviz/assets/iconFont/hide_bar.png" alt="WebRviz" style="height: 1em; vertical-align: middle;"> button collapses or expands the left configuration area.
- The top-left <img src="web_rviz/assets/iconFont/trajectory.png" alt="WebRviz" style="height: 1em; vertical-align: middle;"> button toggles MoveIt planned trajectory preview. After a successful plan, the view shows the trajectory line and TCP start/end pose markers labeled `S / E`.
- The top-left <img src="web_rviz/assets/iconFont/graph.png" alt="WebRviz" style="height: 1em; vertical-align: middle;"> button opens the graph dialog, which supports the detailed node-topic graph, node communication graph, search, refresh, and fit-to-view.
- The top-left <img src="web_rviz/assets/iconFont/data.png" alt="WebRviz" style="height: 1em; vertical-align: middle;"> button opens the data analysis dialog, showing joint/TCP curves from MoveIt planning results with unit switching and hover readouts.
- The top-right <img src="web_rviz/assets/iconFont/reset_view.png" alt="WebRviz" style="height: 1em; vertical-align: middle;"> button restores the default camera view.
- The background, grid, robot material styling, and planned trajectory appearance all follow the current light/dark theme.

## Trajectory Recording and Playback

- `⏺`: enters waiting mode. Actual recording starts when robot motion is detected, and stops automatically when the robot finishes moving.
- `▶`: plays back the recorded trajectory.
- `⏸`: pauses at the current frame and can be resumed.
- `✖`: clears the recorded trajectory.
- Dragging the progress bar pauses playback and jumps to the target frame; the time display on the right updates to the current progress and total duration.

## ROS Graph and Motion Charts

### 1. ROS Graph

- Opened from the top-left <img src="web_rviz/assets/iconFont/graph.png" alt="WebRviz" style="height: 1em; vertical-align: middle;"> button in a separate dialog, with support for `Refresh`, `Fit View`, search, and click-to-inspect details.
- `Detailed Graph` mode shows the full `node -> topic -> node` `Nodes/Topics (all)` relationships, and displays publishing, subscribing, and service details on the right.
- `Node Communication Graph` mode groups communication edges between the same pair of nodes and labels each edge with the related topics for quick inspection of major communication paths.
- Graph data comes from `rosapi` `/rosapi/nodes` and `/rosapi/node_details`, and reuses the topic/type cache to fill in message type information.

### 2. Motion Charts

- Opened from the top-left <img src="web_rviz/assets/iconFont/data.png" alt="WebRviz" style="height: 1em; vertical-align: middle;"> button in a separate dialog. The chart data comes from `planned_trajectory.joint_trajectory.points` inside `/move_group/result`.
- Supports five chart categories: `Joint Trajectory`, `TCP Trajectory`, `Joint Velocity`, `Joint Acceleration`, and `Effort`. If one category has no data, the empty chart frame is still kept.
- Supports unit switching for `deg/rad`, `mm/m`, `deg/s/rad/s`, and `deg/s^2/rad/s^2`, plus joint visibility filtering and `All / Reset` shortcuts.
- When the mouse hovers near a curve, the current time and value are shown for quick reading and comparison.
- This dialog is independent from the bottom `Trajectory Recording and Playback` area: the former is for analyzing MoveIt planning results, and the latter is for local recording and playback of actual motion.

---

## Log Output

- The lower-left log panel records key operations, including connect, sync, record, playback, and TF selection.
- MoveIt-related events are printed with a green `[MoveIt]` prefix, such as planning started, planning finished, execution started, and execution finished.
- The log uses compact line wrapping to remain readable in smaller windows.

---

## `package://` Resource Mapping

### Background

Typical mesh references in URDF:

```xml
<mesh filename="package://urdf_package_name/meshes/base_link.STL"/>
```

A browser cannot directly read local filesystem paths such as `/home/...`, so the resources must be exposed through HTTP URLs.

### How This Project Handles It

The recommended default approach is to start `webrviz_asset_server` on the ROS side. It dynamically resolves any `package://<package_name>/...` request and exposes it over HTTP as:

- URL: `http://<rosbridge-host>:8081/ros_pkgs/<package_name>/...`
- Local resolution: dynamically resolved from the ROS package index in the active environment

Keep the page setting at its default value:

```text
URDF package root URL = /ros_pkgs
```

When the default value `/ros_pkgs` is used, WebRviz automatically derives the remote asset root after connecting to rosbridge.

If the ROS-side asset service is not enabled, you can still mount a local directory manually:

```bash
--mount urdf_package_name=~/your_ws/src/urdf_package_name
```

---

## How the `Sync RViz` Button Works

After clicking `Sync RViz`, WebRviz does the following:

1. Reads the config file specified by `RViz config URL`.
2. Extracts the `Fixed Frame` and candidate point cloud topics.
3. Re-discovers the current ROS topics and message types.
4. Selects an available point cloud topic and rebuilds the subscription.
5. Refreshes the web-side rendering state.

> Note: only data is synchronized. RViz panel layout is not copied.

---

## Qt Integration

It is recommended to open the local page with `QWebEngineView`:

```text
http://127.0.0.1:8080
```

It is also recommended to deploy the following services on the same host:

- ROS master
- rosbridge
- WebRviz static server

This reduces cross-network debugging complexity.

---

## Troubleshooting

### 1) `Unexpected token` when starting the frontend

This is usually caused by an outdated Node.js version. Upgrade to Node 18+.

### 2) rosbridge reports `/rosapi/topics_and_types does not exist`

The current version already handles this case by using `/rosapi/topics` and `/rosapi/topic_type`.

### 3) The model does not show and `*.STL 404` appears

This usually means `package://` resources are not mapped correctly:

- Are you starting the app with `tools/serve_webrviz.py`?
- Did you configure `--mount urdf_package_name=...`?
- Is `URDF package root URL` set to `/ros_pkgs` on the page?

### 4) The model pose does not match RViz

Please confirm:

- `Fixed Frame` matches RViz.
- Click `Sync RViz` and check again.

### 5) Python reports `type object is not subscriptable`

This is a Python syntax compatibility issue, commonly seen with 3.8. The script has already been made compatible with Python 3.8.
