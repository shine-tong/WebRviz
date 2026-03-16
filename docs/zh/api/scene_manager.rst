场景管理模块（sceneManager.ts）
===============================

模块路径：``web_rviz/src/visualization/sceneManager.ts``

用途
----

维护 Three.js 场景、机器人姿态、TF 数据、主题联动的视口样式与点云显示。

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

SceneManager 核心方法
---------------------

- ``setTargetFps(value)``：设置渲染目标帧率。
- ``setTheme(theme)``：根据界面主题更新中间 3D 视图的背景色与网格颜色。
- ``setFixedFrame(frame)`` / ``getFixedFrame()``：设置和读取固定坐标系。
- ``getRobotBaseFrame()``：获取当前机器人基坐标系。
- ``setEndEffectorFrame(frame)``：设置末端执行器坐标系。
- ``setShowOnlyEndEffector(show)``：切换末端预览模式。
- ``setVisibleTfFrames(frames)``：限制可见 TF 坐标轴集合；传入 ``null`` 可恢复不过滤状态。
- ``getDefaultEndEffectorFrame()``：推断默认末端坐标系。
- ``getLinkList()`` / ``getFrameList()``：获取连杆与帧列表。
- ``getTfSnapshot()`` / ``getTfNodes()``：读取 TF 快照信息。
- ``clearTfRecords()``：清空 TF 记录并刷新显示。
- ``getRelativeTransform(frame)``：获取相对固定坐标系的位姿。
- ``setRobot(robot, baseFrame)``：设置机器人模型。
- ``updateJointStates(message)``：应用关节状态到机器人模型。
- ``upsertTfMessage(message)``：增量更新 TF 缓存。
- ``setPointCloud(pointCloud)``：更新点云渲染对象。
- ``dispose()``：释放渲染资源与监听器。

使用建议
--------

- ``dispose()`` 应在页面卸载时调用，避免 WebGL 资源泄漏。
- 主题切换时应同步调用 ``setTheme()``，确保视口背景与网格颜色和界面风格一致。
- 需要按需显示 TF 时，优先使用 ``setVisibleTfFrames()``，而不是直接改动 TF 树文本数据。
- 高并发点云场景下，建议配合 ``maxPoints`` 与 ``targetFps`` 控制性能。
