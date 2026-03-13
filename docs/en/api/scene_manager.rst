Scene Manager Module (sceneManager.ts)
======================================

Module path: ``web_rviz/src/visualization/sceneManager.ts``

Purpose
-------

Owns the Three.js scene, robot state updates, TF cache, and point cloud rendering.

Public type
-----------

``TfNode``
^^^^^^^^^^

- ``frame``: current frame id.
- ``parent``: parent frame id.
- ``translation``: translation vector.
- ``rotation``: quaternion rotation.

SceneManager key methods
------------------------

- ``setTargetFps(value)``: updates render target FPS.
- ``setFixedFrame(frame)`` / ``getFixedFrame()``: fixed frame setter/getter.
- ``setEndEffectorFrame(frame)``: sets end-effector frame.
- ``setShowOnlyEndEffector(show)``: toggles end-effector-only preview.
- ``getDefaultEndEffectorFrame()``: infers a default end-effector frame.
- ``getLinkList()`` / ``getFrameList()``: lists links and known frames.
- ``getTfSnapshot()`` / ``getTfNodes()``: reads TF graph snapshots.
- ``clearTfRecords()``: clears TF cache and visuals.
- ``getRelativeTransform(frame)``: resolves pose relative to fixed frame.
- ``setRobot(robot, baseFrame)``: sets/updates robot model.
- ``updateJointStates(message)``: applies joint state messages.
- ``upsertTfMessage(message)``: merges TF updates.
- ``setPointCloud(pointCloud)``: updates point cloud object.
- ``dispose()``: releases render resources and listeners.

Usage notes
-----------

- Call ``dispose()`` before page teardown to avoid WebGL leaks.
- For high-rate point clouds, tune ``maxPoints`` and ``targetFps`` together.
