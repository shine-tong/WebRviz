运行时配置模块（config.ts）
===========================

模块路径：``web_rviz/src/config.ts``

用途
----

负责 WebRviz 在浏览器端的运行时配置读取与持久化。

公开接口
--------

``RuntimeConfig``
^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1

   * - 字段
     - 类型
     - 说明
   * - ``rosbridgeUrl``
     - ``string``
     - ROS WebSocket 地址。
   * - ``rvizConfigPath``
     - ``string``
     - RViz 配置文件 URL。
   * - ``urdfFallbackPath``
     - ``string``
     - ``/robot_description`` 不可用时的 URDF 回退路径。
   * - ``packageRootUrl``
     - ``string``
     - ``package://`` 资源映射的 HTTP 根路径。
   * - ``fixedFrame``
     - ``string``
     - 场景固定坐标系。
   * - ``defaultPointCloudTopic``
     - ``string``
     - 默认点云话题。
   * - ``maxPoints``
     - ``number``
     - 点云解码后保留的最大点数量。
   * - ``targetFps``
     - ``number``
     - 目标渲染帧率。

``defaultConfig``
^^^^^^^^^^^^^^^^^

提供默认配置对象，首次访问或本地缓存异常时作为回退值。

``loadConfig(): RuntimeConfig``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- 从 ``localStorage`` 读取键 ``webrviz-runtime-config``。
- 读取失败或 JSON 解析失败时回退到 ``defaultConfig``。

``saveConfig(config: RuntimeConfig): void``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

将配置序列化为 JSON 并写入 ``localStorage``。
