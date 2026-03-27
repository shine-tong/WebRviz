ROS 资源服务模块（serve_ros_packages.py）
=========================================

模块路径：``webrviz_asset_server/scripts/serve_ros_packages.py``

关联 launch 文件：``webrviz_asset_server/launch/asset_server.launch``

用途
----

将 URDF 中 ``package://`` 引用的 ROS 包资源通过 HTTP 暴露给 WebRviz，使浏览器可以从 ROS 主机加载 mesh 与贴图文件。

公开行为
--------

HTTP 路由
^^^^^^^^^

- ``GET /ros_pkgs/<package>/<path>``：通过 ``rospkg.RosPack`` 动态解析 ``<package>``，并返回对应文件。
- ``HEAD /ros_pkgs/<package>/<path>``：返回相同资源的头信息。
- ``OPTIONS /ros_pkgs/<package>/<path>``：返回浏览器跨域预检所需的 CORS 响应头。
- ``GET /healthz``：健康检查接口，返回 ``ok``。

解析规则
^^^^^^^^

- 包路径从当前 ROS 环境动态解析，不写死机器人包名。
- 已解析的包目录会按包名缓存。
- 含 ``..`` 或越出包目录边界的请求会被拒绝。
- 包不存在或文件不存在时返回 ``404``。

服务行为
^^^^^^^^

- 添加 ``Access-Control-Allow-Origin: *``，支持浏览器跨域加载模型资源。
- 根据文件扩展名推断 ``Content-Type``，无法识别时回退到 ``application/octet-stream``。
- 忽略 ``roslaunch`` 自动附带的 ``__name:=...``、``__log:=...`` 等 remap 参数。

命令行参数
^^^^^^^^^^

``--host``
  HTTP 服务监听地址，默认值为 ``0.0.0.0``。

``--port``
  HTTP 服务监听端口，默认值为 ``8081``。

Launch 集成
-----------

``asset_server.launch`` 会以 ROS 节点方式启动该脚本，并透传两个 launch 参数：

- ``host``：传给 ``--host`` 的监听地址。
- ``port``：传给 ``--port`` 的监听端口。

典型部署中，该服务与 ``rosbridge`` 运行在同一台 ROS 主机上，前端连接后即可自动推导出
``http(s)://<rosbridge-host>:8081/ros_pkgs`` 作为运行时资源根地址。
