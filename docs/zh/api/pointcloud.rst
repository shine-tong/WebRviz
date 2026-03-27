点云解码模块（pointcloud.ts）
=============================

模块路径：``web_rviz/src/ros/pointcloud.ts``

用途
----

将 ``sensor_msgs/PointCloud2`` 消息转换为 Three.js 可直接消费的点位与颜色数组。

公开接口
--------

``ParsedPointCloud``
^^^^^^^^^^^^^^^^^^^^

- ``frameId``：点云参考坐标系。
- ``positions``：``Float32Array``，长度为 ``count * 3``。
- ``colors``：``Float32Array``，长度为 ``count * 3``。
- ``count``：有效点数量。

``decodePointCloud2(rawMessage: unknown, maxPoints: number): ParsedPointCloud | null``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

行为说明：

- 自动处理 ``base64`` 字符串或 ``number[]`` 二进制载荷。
- 支持 ``x/y/z`` 必需字段；缺失时返回 ``null``。
- 优先使用 ``rgb/rgba`` 字段，前提是颜色数据本身具有有效区分度。
- 若缺少可用颜色，则按 ``z`` 深度范围生成接近 RViz 的深度渐变色；当深度范围过小无法区分时，再退化为按点到原点距离着色。
- 自动按 ``maxPoints`` 抽样以控制渲染负载。
- 忽略 ``NaN`` 与非法点。
