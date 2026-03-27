Trajectory Chart Renderer Module (trajectoryCharts.ts)
======================================================

Module path: ``web_rviz/src/trajectoryCharts.ts``

Purpose
-------

Renders interactive motion-segment line charts on ``canvas`` elements for the trajectory modal.

Public API
----------

``TrajectoryChartSeries``
^^^^^^^^^^^^^^^^^^^^^^^^^

- ``label``: legend label shown in hover state.
- ``color``: stroke color for the series.
- ``values``: sampled numeric values aligned with ``times``.
- ``visible?``: optional flag to exclude the series from drawing and range calculation.

``TrajectoryChartTheme``
^^^^^^^^^^^^^^^^^^^^^^^^

- ``background``: chart background fill.
- ``border``: outer frame color.
- ``grid``: grid line color.
- ``text``: primary text color.
- ``muted``: secondary label color.
- ``accent``: playhead and hover guideline color.

``TrajectoryLineChartOptions``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- ``canvas``: target ``HTMLCanvasElement``.
- ``times``: sample times in milliseconds.
- ``series``: list of chart series.
- ``unitLabel``: unit text rendered next to the y-axis labels.
- ``emptyLabel``: fallback label when no finite data is available.
- ``theme``: colors used for drawing.
- ``playheadTime?``: optional highlighted time cursor in milliseconds.

``renderTrajectoryLineChart(options: TrajectoryLineChartOptions): void``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Draws or refreshes the chart bound to ``options.canvas``.

Behavior Notes
--------------

- Keeps one interactive state object per canvas via ``WeakMap``; repeated calls update the existing chart instance.
- Resizes the backing canvas to match CSS size and caps device pixel ratio at ``2``.
- Decimates dense series before drawing to keep hover and repaint work fast.
- Ignores hidden series and non-finite values when computing chart bounds.
- Supports hover tooltips, nearest-series selection, and an optional playhead guide line.
