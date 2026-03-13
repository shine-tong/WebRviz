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
- Maps ``package://`` to HTTP package root URL.
- Returns a ``LoadedRobotModel``.

``loadRobotModel(rosClient, fallbackPath, packageRootUrl)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

One-shot helper that fetches URDF and builds the robot object.
