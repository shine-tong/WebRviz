Scene Manager Module (sceneManager.ts)
======================================

Module path: ``web_rviz/src/visualization/sceneManager.ts``

Purpose
-------

Owns the Three.js scene, robot state updates, TF cache, theme-aware viewport styling,
planned-trajectory preview rendering, motion-sampling transforms, and point cloud rendering.

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

``RobotJointConnection``
^^^^^^^^^^^^^^^^^^^^^^^^

- ``name``: joint name from the loaded URDF robot.
- ``parent``: parent link name.
- ``child``: child link name.
- ``type``: URDF joint type such as ``fixed`` or ``revolute``.

``RobotTransformDetail``
^^^^^^^^^^^^^^^^^^^^^^^^

- ``xyz``: translation in meters.
- ``rpy``: roll, pitch, yaw in radians.

``RobotInertiaDetail``
^^^^^^^^^^^^^^^^^^^^^^

- ``ixx`` / ``ixy`` / ``ixz`` / ``iyy`` / ``iyz`` / ``izz``: inertia tensor entries when available.

``RobotLinkDetail``
^^^^^^^^^^^^^^^^^^^

- ``name``: link name.
- ``parentJoint``: parent joint name, or ``null`` for the base link.
- ``childJoints``: child joint names.
- ``visualCount`` / ``collisionCount``: number of URDF visual and collision elements.
- ``materialNames``: referenced visual material names.
- ``mass``: inertial mass if present.
- ``inertialOrigin`` / ``inertia``: inertial pose and inertia tensor parsed from URDF.
- ``pose``: current pose of the link relative to the active fixed frame.

``RobotJointDetail``
^^^^^^^^^^^^^^^^^^^^

- ``name`` / ``type``: joint identifier and URDF type.
- ``parentLink`` / ``childLink``: connected links.
- ``axis``: joint axis, or ``null`` when not provided.
- ``origin``: URDF joint origin.
- ``limit``: lower, upper, effort, and velocity limits when present.
- ``dynamics``: damping and friction values when present.
- ``mimic``: mimic source, multiplier, and offset when present.
- ``currentValue``: current runtime joint value array.

SceneManager key methods
------------------------

- ``setTargetFps(value)``: updates render target FPS.
- ``resetView()``: restores the default camera position and OrbitControls state.
- ``setTheme(theme)``: updates viewport background, grid colors, lighting, and theme-aware scene styling.
- ``setPlannedTrajectoryVisible(visible)``: toggles the MoveIt planned-trajectory preview.
- ``setPlannedTrajectoryPath(points, startPose?, endPose?)``: renders the planned path polyline and optional start/end TCP pose markers.
- ``clearPlannedTrajectory()``: removes the planned path and its pose markers from the scene.
- ``setFixedFrame(frame)`` / ``getFixedFrame()``: fixed frame setter/getter.
- ``getRobotBaseFrame()``: returns the active robot base frame.
- ``setEndEffectorFrame(frame)``: sets end-effector frame.
- ``setShowOnlyEndEffector(show)``: toggles end-effector-only preview.
- ``setVisibleTfFrames(frames)``: limits visible TF axes to the provided frame ids, or resets filtering with ``null``.
- ``getDefaultEndEffectorFrame()``: infers a default end-effector frame from the loaded robot model.
- ``getLinkList()`` / ``getFrameList()``: lists robot links and known frames.
- ``getRobotJointConnections()``: returns sorted parent/child joint edges extracted from the loaded URDF robot.
- ``getTfSnapshot()`` / ``getTfNodes()``: reads TF graph snapshots.
- ``clearTfRecords()``: clears TF cache and visuals.
- ``getRelativeTransform(frame)``: resolves a pose relative to the fixed frame, preferring live TF when available.
- ``getRobotRelativeTransform(frame)``: resolves a pose directly from the robot model without consulting live TF.
- ``getRobotLinkDetail(linkName)``: returns structured link metadata for the TF detail modal.
- ``getRobotJointDetail(jointName)``: returns structured joint metadata for the TF detail modal.
- ``setRobot(robot, baseFrame)``: sets or updates the robot model.
- ``updateJointStates(message)``: applies joint state messages.
- ``upsertTfMessage(message)``: merges TF updates.
- ``setPointCloud(pointCloud)``: updates the point cloud object.
- ``dispose()``: releases render resources and listeners.

Usage notes
-----------

- Call ``dispose()`` before page teardown to avoid WebGL leaks.
- Use ``setTheme()`` together with the UI theme toggle so the viewport background and grid stay visually consistent.
- Use ``setVisibleTfFrames()`` to support selective TF visualization without mutating the TF tree text data.
- Use ``getRobotRelativeTransform()`` when sampling planned motion from robot joint values or converting MoveIt joint trajectory points into TCP curves; it avoids mixing preview data with live TF.
- Use ``setPlannedTrajectoryPath()`` with ``startPose`` and ``endPose`` to show both the preview line and TCP start/end pose markers.
- For high-rate point clouds, tune ``maxPoints`` and ``targetFps`` together.
