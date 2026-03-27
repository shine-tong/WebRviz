ROS Asset Server Module (serve_ros_packages.py)
===============================================

Module path: ``webrviz_asset_server/scripts/serve_ros_packages.py``

Related launch file: ``webrviz_asset_server/launch/asset_server.launch``

Purpose
-------

Exposes ROS package assets referenced by ``package://`` URDF URLs as HTTP files that WebRviz can fetch from the ROS host.

Public Behavior
---------------

HTTP routes
^^^^^^^^^^^

- ``GET /ros_pkgs/<package>/<path>``: resolves ``<package>`` via ``rospkg.RosPack`` and serves the requested file.
- ``HEAD /ros_pkgs/<package>/<path>``: metadata-only variant of the same asset route.
- ``OPTIONS /ros_pkgs/<package>/<path>``: returns CORS headers for browser preflight compatibility.
- ``GET /healthz``: lightweight health check that returns ``ok``.

Resolution rules
^^^^^^^^^^^^^^^^

- Package paths are resolved dynamically from the active ROS environment.
- Resolved package directories are cached per package name.
- Requests that escape the package directory via ``..`` or invalid normalization are rejected.
- Missing packages or missing files return ``404``.

Serving behavior
^^^^^^^^^^^^^^^^

- Adds ``Access-Control-Allow-Origin: *`` so browser clients can load meshes cross-origin.
- Guesses content type from file extension and falls back to ``application/octet-stream``.
- Ignores extra ROS launch remap arguments such as ``__name:=...`` and ``__log:=...``.

Command-line arguments
^^^^^^^^^^^^^^^^^^^^^^

``--host``
  Bind address for the HTTP server. Default: ``0.0.0.0``.

``--port``
  Bind port for the HTTP server. Default: ``8081``.

Launch integration
------------------

``asset_server.launch`` starts the script as a ROS node and forwards two launch arguments:

- ``host``: bind address passed to ``--host``.
- ``port``: bind port passed to ``--port``.

Typical deployment keeps this server on the same ROS host as ``rosbridge`` so the frontend can derive
``http(s)://<rosbridge-host>:8081/ros_pkgs`` at connection time.
