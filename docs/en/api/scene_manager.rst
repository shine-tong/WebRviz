Scene Manager Module (sceneManager.ts)
======================================

Module path: ``web_rviz/src/visualization/sceneManager.ts``

Purpose
-------

Owns the Three.js scene, robot state updates, TF cache, theme-aware viewport styling, and point cloud rendering.

Public types
------------

``SceneTheme``
^^^^^^^^^^^^^^

- ``dark``
- ``light``

``TfNode``
^^^^^^^^^^

- ``frame``: current frame id.
- ``parent``: parent frame id.
- ``translation``: translation vector.
- ``rotation``: quaternion rotation.

SceneManager key methods
------------------------

- ``setTargetFps(value)``: updates render target FPS.
- ``setTheme(theme)``: updates viewport background and grid colors for the current UI theme.
- ``setFixedFrame(frame)`` / ``getFixedFrame()``: fixed frame setter/getter.
- ``getRobotBaseFrame()``: returns the active robot base frame.
- ``setEndEffectorFrame(frame)``: sets end-effector frame.
- ``setShowOnlyEndEffector(show)``: toggles end-effector-only preview.
- ``setVisibleTfFrames(frames)``: limits visible TF axes to the provided frame ids, or resets filtering with ``null``.
- ``getDefaultEndEffectorFrame()``: infers a default end-effector frame.
- ``getLinkList()`` / ``getFrameList()``: lists links and known frames.
- ``getTfSnapshot()`` / ``getTfNodes()``: reads TF graph snapshots.
- ``clearTfRecords()``: clears TF cache and visuals.
- ``getRelativeTransform(frame)``: resolves pose relative to fixed frame.
- ``setRobot(robot, baseFrame)``: sets or updates the robot model.
- ``updateJointStates(message)``: applies joint state messages.
- ``upsertTfMessage(message)``: merges TF updates.
- ``setPointCloud(pointCloud)``: updates point cloud object.
- ``dispose()``: releases render resources and listeners.

Usage notes
-----------

- Call ``dispose()`` before page teardown to avoid WebGL leaks.
- Use ``setTheme()`` together with the UI theme toggle so the viewport background and grid stay visually consistent.
- Use ``setVisibleTfFrames()`` to support selective TF visualization without mutating the TF tree text data.
- For high-rate point clouds, tune ``maxPoints`` and ``targetFps`` together.
