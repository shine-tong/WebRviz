场景管理模块（sceneManager.ts）
===============================

模块路径：``web_rviz/src/visualization/sceneManager.ts``

用途
----

维护 Three.js 场景、机器人姿态、TF 数据、主题联动的视口样式、规划轨迹预览、运动采样位姿计算与点云显示。

公开类型
--------

``SceneTheme``
^^^^^^^^^^^^^^

- ``dark``
- ``light``

``TfNode``
^^^^^^^^^^

- ``frame``：当前坐标系。
- ``parent``：父坐标系。
- ``translation``：平移向量。
- ``rotation``：旋转四元数。

``RobotJointConnection``
^^^^^^^^^^^^^^^^^^^^^^^^

- ``name``：URDF 中的关节名称。
- ``parent``：父 link 名称。
- ``child``：子 link 名称。
- ``type``：URDF 关节类型，例如 ``fixed``、``revolute``。

``RobotTransformDetail``
^^^^^^^^^^^^^^^^^^^^^^^^

- ``xyz``：平移，单位为米。
- ``rpy``：滚转、俯仰、偏航，单位为弧度。

``RobotInertiaDetail``
^^^^^^^^^^^^^^^^^^^^^^

- ``ixx`` / ``ixy`` / ``ixz`` / ``iyy`` / ``iyz`` / ``izz``：若存在则返回惯量张量分量。

``RobotLinkDetail``
^^^^^^^^^^^^^^^^^^^

- ``name``：link 名称。
- ``parentJoint``：父 joint 名称；基座 link 时为 ``null``。
- ``childJoints``：子 joint 名称列表。
- ``visualCount`` / ``collisionCount``：URDF 中 visual 与 collision 元素数量。
- ``materialNames``：visual 中引用的材质名列表。
- ``mass``：若存在则返回惯性质量。
- ``inertialOrigin`` / ``inertia``：从 URDF 解析出的惯性原点与惯量信息。
- ``pose``：当前相对固定坐标系的位姿。

``RobotJointDetail``
^^^^^^^^^^^^^^^^^^^^

- ``name`` / ``type``：joint 名称与 URDF 类型。
- ``parentLink`` / ``childLink``：连接的父子 link。
- ``axis``：关节轴；未提供时为 ``null``。
- ``origin``：URDF joint 原点。
- ``limit``：若存在则包含上下限、力矩和速度限制。
- ``dynamics``：若存在则包含阻尼和摩擦参数。
- ``mimic``：若存在则包含 mimic 源关节、倍数和偏移。
- ``currentValue``：运行时当前关节值数组。

SceneManager 核心方法
---------------------

- ``setTargetFps(value)``：设置渲染目标帧率。
- ``resetView()``：恢复默认相机位置与 OrbitControls 视角状态。
- ``setTheme(theme)``：根据界面主题更新背景、网格、光照和主题相关的场景样式。
- ``setPlannedTrajectoryVisible(visible)``：切换 MoveIt 规划轨迹预览的显示状态。
- ``setPlannedTrajectoryPath(points, startPose?, endPose?)``：渲染规划轨迹折线，并可选显示 TCP 起点/终点姿态标记。
- ``clearPlannedTrajectory()``：清除规划轨迹折线及起终点姿态标记。
- ``setFixedFrame(frame)`` / ``getFixedFrame()``：设置和读取固定坐标系。
- ``getRobotBaseFrame()``：获取当前机器人基坐标系。
- ``setEndEffectorFrame(frame)``：设置末端执行器坐标系。
- ``setShowOnlyEndEffector(show)``：切换末端预览模式。
- ``setVisibleTfFrames(frames)``：限制可见 TF 坐标轴集合；传入 ``null`` 可恢复不过滤状态。
- ``getDefaultEndEffectorFrame()``：根据已加载的机器人模型推断默认末端坐标系。
- ``getLinkList()`` / ``getFrameList()``：获取机器人 link 列表与已知 frame 列表。
- ``getTfSnapshot()`` / ``getTfNodes()``：读取 TF 图快照信息。
- ``clearTfRecords()``：清空 TF 记录并刷新显示。
- ``getRelativeTransform(frame)``：获取相对固定坐标系的位姿，优先使用实时 TF。
- ``getRobotRelativeTransform(frame)``：直接从机器人模型求取相对位姿，不依赖实时 TF。
- ``getRobotLinkDetail(linkName)``：返回 TF 详情弹窗使用的 link 结构化信息。
- ``getRobotJointDetail(jointName)``：返回 TF 详情弹窗使用的 joint 结构化信息。
- ``setRobot(robot, baseFrame)``：设置或更新机器人模型。
- ``updateJointStates(message)``：将关节状态应用到机器人模型。
- ``upsertTfMessage(message)``：增量更新 TF 缓存。
- ``setPointCloud(pointCloud)``：更新点云渲染对象。
- ``dispose()``：释放渲染资源与监听器。

使用建议
--------

- 页面卸载前应调用 ``dispose()``，避免 WebGL 资源泄漏。
- 主题切换时应同步调用 ``setTheme()``，确保视口背景与网格颜色和界面风格一致。
- 需要按需显示 TF 时，优先使用 ``setVisibleTfFrames()``，而不是直接改动 TF 树文本数据。
- 使用关节采样来预览规划轨迹，或根据 MoveIt 关节轨迹点换算 TCP 曲线时，优先调用 ``getRobotRelativeTransform()``，避免把预览轨迹和实时 TF 混在一起。
- 使用 ``setPlannedTrajectoryPath()`` 时可同时传入 ``startPose`` 和 ``endPose``，在预览路径的同时显示 TCP 起终点姿态。
- 高并发点云场景下，建议配合 ``maxPoints`` 与 ``targetFps`` 控制性能。
