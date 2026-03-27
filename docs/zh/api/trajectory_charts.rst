运动曲线渲染模块（trajectoryCharts.ts）
========================================

模块路径：``web_rviz/src/trajectoryCharts.ts``

用途
----

在轨迹弹窗中基于 ``canvas`` 渲染可交互的运动段折线图。

公开接口
--------

``TrajectoryChartSeries``
^^^^^^^^^^^^^^^^^^^^^^^^^

- ``label``：悬停提示中显示的序列名称。
- ``color``：该序列的折线颜色。
- ``values``：与 ``times`` 对齐的采样数值。
- ``visible?``：可选，设为 ``false`` 时不参与绘制和范围计算。

``TrajectoryChartTheme``
^^^^^^^^^^^^^^^^^^^^^^^^

- ``background``：图表背景色。
- ``border``：外框颜色。
- ``grid``：网格线颜色。
- ``text``：主文本颜色。
- ``muted``：次级标签颜色。
- ``accent``：播放头与悬停辅助线颜色。

``TrajectoryLineChartOptions``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- ``canvas``：目标 ``HTMLCanvasElement``。
- ``times``：毫秒级采样时间数组。
- ``series``：曲线序列列表。
- ``unitLabel``：绘制在纵轴旁的单位文本。
- ``emptyLabel``：无有效数据时显示的占位文本。
- ``theme``：绘图使用的颜色集合。
- ``playheadTime?``：可选，毫秒级播放头位置。

``renderTrajectoryLineChart(options: TrajectoryLineChartOptions): void``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

绘制或刷新绑定到 ``options.canvas`` 的图表。

行为说明
--------

- 通过 ``WeakMap`` 为每个 canvas 保持一份交互状态；重复调用会复用已有图表实例。
- 自动把底层 canvas 尺寸同步到 CSS 显示尺寸，并将设备像素比上限限制为 ``2``。
- 在绘制前会对高密度序列做抽稀，降低重绘与悬停计算开销。
- 计算坐标范围时会忽略隐藏序列和非有限值。
- 支持悬停 tooltip、最近序列命中，以及可选的播放头参考线。
