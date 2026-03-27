Robot Loader Module (robotLoader.ts)
====================================

Module path: ``web_rviz/src/visualization/robotLoader.ts``

Purpose
-------

Loads URDF text and builds a renderable robot model via ``urdf-loader``.

Public API
----------

``LoadedRobotModel``
^^^^^^^^^^^^^^^^^^^^

- ``robot``: parsed urdf-loader robot object.
- ``rootLink``: inferred root link name.

``loadRobotUrdfText(rosClient, fallbackPath)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Reads URDF from ``/robot_description`` first, then falls back to URL.

``buildRobotFromUrdf(urdfXml, packageRootUrl)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Validates XML.
- Infers root link.
- Maps ``package://<package>`` to ``<packageRootUrl>/<package>``.
- Returns a ``LoadedRobotModel``.

``loadRobotModel(rosClient, fallbackPath, packageRootUrl)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

One-shot helper that fetches URDF and builds the robot object.

- Callers are expected to pass the effective runtime asset root, not just the persisted config value.
- In the default connect flow, this runtime root can be auto-derived from the rosbridge host so meshes
  are fetched from the ROS-side asset server.
