机器人加载模块（robotLoader.ts）
================================

模块路径：``web_rviz/src/visualization/robotLoader.ts``

用途
----

加载 URDF 文本并构建 ``urdf-loader`` 可渲染机器人对象。

公开接口
--------

``LoadedRobotModel``
^^^^^^^^^^^^^^^^^^^^

- ``robot``：urdf-loader 解析出的机器人对象。
- ``rootLink``：推断出的根连杆名。

``loadRobotUrdfText(rosClient, fallbackPath)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

优先从 ``/robot_description`` 获取 URDF，失败时使用回退 URL。

``buildRobotFromUrdf(urdfXml, packageRootUrl)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- 校验 XML 合法性。
- 推断根连杆。
- 将 ``package://<package>`` 映射为 ``<packageRootUrl>/<package>``。
- 返回 ``LoadedRobotModel``。

``loadRobotModel(rosClient, fallbackPath, packageRootUrl)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

一站式完成 URDF 获取与机器人构建。

- 调用方应传入当前连接实际生效的运行时资源根，而不只是持久化配置值。
- 默认连接流程下，该运行时资源根可根据 rosbridge 主机地址自动推导，从而让 mesh
  资源直接从 ROS 侧资源服务加载。
