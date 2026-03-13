RViz 配置同步模块（rvizConfig.ts）
===================================

模块路径：``web_rviz/src/rviz/rvizConfig.ts``

用途
----

解析 RViz 配置（YAML），提取固定坐标系和点云话题建议值。

公开接口
--------

``RvizSyncHints``
^^^^^^^^^^^^^^^^^

- ``fixedFrame?: string``
- ``pointCloudTopics: string[]``

``parseRvizConfig(content: string): RvizSyncHints``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- 解析 ``Visualization Manager`` 节点。
- 从 ``Global Options`` 中读取 ``Fixed Frame``。
- 递归扫描 ``Displays``，收集 ``PointCloud`` 相关显示项的话题。

``loadRvizSyncHints(url: string): Promise<RvizSyncHints>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- 通过 ``fetch`` 拉取远程 RViz 配置。
- HTTP 非 2xx 时抛出错误。
- 成功后调用 ``parseRvizConfig`` 返回结果。
