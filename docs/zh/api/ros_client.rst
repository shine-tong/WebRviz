ROS 客户端模块（rosClient.ts）
==============================

模块路径：``web_rviz/src/ros/rosClient.ts``

用途
----

封装 ``roslib`` 的连接、话题创建、以及 ``rosapi`` 服务调用，并为 `ROS Info` 面板和 `ROS 图谱` 快照提供数据访问能力。

核心类型
--------

- ``ConnectionState``：``disconnected | connecting | connected | error``
- ``TopicInfo``：话题名称与类型。
- ``ServiceInfo``：服务名称与类型。
- ``MessageTypeDef``：消息字段定义。
- ``MessageDetailsResponse``：消息详情响应。
- ``ServiceDetailsResponse``：服务请求/响应详情。
- ``NodeDetails``：节点的发布话题、订阅话题与服务列表。

RosClient
---------

``onStateChange(listener)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^

注册连接状态监听器。

``getState()`` / ``isConnected()``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

获取当前状态与连接布尔值。

``connect(url: string): Promise<void>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- 建立 ROS WebSocket 连接。
- 内部处理 ``connection``、``error``、``close`` 事件。
- 连接失败会抛出异常。

``disconnect(): void``
^^^^^^^^^^^^^^^^^^^^^^

关闭连接并重置状态。

``createTopic(name, messageType, throttleRateMs = 0)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

创建 ``ROSLIB.Topic``，用于订阅或发布。

``listTopicsWithTypes()``
^^^^^^^^^^^^^^^^^^^^^^^^^

通过 ``/rosapi/topics`` + ``/rosapi/topic_type`` 获取完整话题列表。

``listServicesWithTypes()``
^^^^^^^^^^^^^^^^^^^^^^^^^^^

通过 ``/rosapi/services`` + ``/rosapi/service_type`` 获取完整服务列表。

``listParams()``
^^^^^^^^^^^^^^^^

通过 ``/rosapi/get_param_names`` 返回参数名数组。

``listNodes()``
^^^^^^^^^^^^^^^

通过 ``/rosapi/nodes`` 返回节点名称列表，供 `ROS 图谱` 构建快照使用。

``getNodeDetails(node)``
^^^^^^^^^^^^^^^^^^^^^^^^

通过 ``/rosapi/node_details`` 返回指定节点的发布、订阅与 service 信息。

``getParam(name)``
^^^^^^^^^^^^^^^^^^

调用 ``/rosapi/get_param`` 返回参数值字符串。

``getMessageDetails(type)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^

调用 ``/rosapi/message_details`` 查询消息类型结构。

``getServiceDetails(type)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^

并行查询请求/响应详情，自动兼容 ``type`` 与 ``service`` 参数差异。
