运行时配置模块（config.ts）
===========================

模块路径：``web_rviz/src/config.ts``

用途
----

负责 WebRviz 在浏览器端的运行时配置读取与持久化。连接 ROS 时，默认的
``packageRootUrl`` 还可能被转换为针对当前 ROS 主机的运行时资源根地址。

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
     - ``package://`` 资源映射的 HTTP 根路径；持久化默认值为 ``/ros_pkgs``。
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

- ``packageRootUrl`` 的默认值为 ``/ros_pkgs``。
- 当调用方保留该默认值时，``main.ts`` 的连接流程会将运行时资源根自动推导为
  ``http(s)://<rosbridge-host>:8081/ros_pkgs``。

``loadConfig(): RuntimeConfig``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- 从 ``localStorage`` 读取键 ``webrviz-runtime-config``。
- 读取失败或 JSON 解析失败时回退到 ``defaultConfig``。

``saveConfig(config: RuntimeConfig): void``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

将配置序列化为 JSON 并写入 ``localStorage``。

行为说明
--------

- 自动推导出的运行时资源根只在当前连接内使用，不要求覆盖持久化的 ``packageRootUrl``。
- 如果用户手工填写了自定义 ``packageRootUrl``，则优先使用该值，并跳过自动推导。
