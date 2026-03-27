Point Cloud Decoder Module (pointcloud.ts)
==========================================

Module path: ``web_rviz/src/ros/pointcloud.ts``

Purpose
-------

Converts ``sensor_msgs/PointCloud2`` payloads into Three.js-ready position/color buffers.

Public API
----------

``ParsedPointCloud``
^^^^^^^^^^^^^^^^^^^^

- ``frameId``: source frame id.
- ``positions``: ``Float32Array`` with length ``count * 3``.
- ``colors``: ``Float32Array`` with length ``count * 3``.
- ``count``: valid decoded point count.

``decodePointCloud2(rawMessage: unknown, maxPoints: number): ParsedPointCloud | null``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Behavior:

- Supports both base64 and ``number[]`` binary payloads.
- Requires ``x/y/z`` fields; returns ``null`` when missing.
- Uses ``rgb/rgba`` if present and meaningfully varied.
- Otherwise falls back to RViz-style depth coloring based on the decoded ``z`` range, with distance-based coloring as the last fallback when the depth range collapses.
- Samples points according to ``maxPoints`` for stable rendering performance.
- Skips invalid numeric values.
