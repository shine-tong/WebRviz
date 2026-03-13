Runtime Config Module (config.ts)
=================================

Module path: ``web_rviz/src/config.ts``

Purpose
-------

Handles browser-side runtime configuration load/save behavior.

Public API
----------

``RuntimeConfig``
^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1

   * - Field
     - Type
     - Description
   * - ``rosbridgeUrl``
     - ``string``
     - ROS bridge WebSocket URL.
   * - ``rvizConfigPath``
     - ``string``
     - RViz config URL.
   * - ``urdfFallbackPath``
     - ``string``
     - URDF fallback path when ``/robot_description`` is unavailable.
   * - ``packageRootUrl``
     - ``string``
     - HTTP root path for ``package://`` resources.
   * - ``fixedFrame``
     - ``string``
     - Scene fixed frame.
   * - ``defaultPointCloudTopic``
     - ``string``
     - Default point cloud topic name.
   * - ``maxPoints``
     - ``number``
     - Max sampled points to keep for rendering.
   * - ``targetFps``
     - ``number``
     - Render target FPS.

``defaultConfig``
^^^^^^^^^^^^^^^^^

Provides default values for first-time load or invalid local cache.

``loadConfig(): RuntimeConfig``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Loads ``webrviz-runtime-config`` from ``localStorage`` and falls back to defaults on parse errors.

``saveConfig(config: RuntimeConfig): void``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Serializes and persists runtime config to ``localStorage``.
