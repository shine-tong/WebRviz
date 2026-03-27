Runtime Config Module (config.ts)
=================================

Module path: ``web_rviz/src/config.ts``

Purpose
-------

Handles browser-side runtime configuration load/save behavior. During connection setup, the default
``packageRootUrl`` value can be translated into a ROS-host-specific runtime asset root.

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
     - HTTP root path for ``package://`` resources. The persisted default is ``/ros_pkgs``.
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

- ``packageRootUrl`` defaults to ``/ros_pkgs``.
- When callers keep that default, the connect flow in ``main.ts`` may derive the effective runtime URL
  as ``http(s)://<rosbridge-host>:8081/ros_pkgs``.

``loadConfig(): RuntimeConfig``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Loads ``webrviz-runtime-config`` from ``localStorage`` and falls back to defaults on parse errors.

``saveConfig(config: RuntimeConfig): void``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Serializes and persists runtime config to ``localStorage``.

Behavior Notes
--------------

- Auto-derived runtime asset roots are connection-scoped helpers; they do not need to overwrite the
  persisted ``packageRootUrl`` value.
- If a user enters a custom ``packageRootUrl``, that explicit value is preserved and auto-derivation is skipped.
