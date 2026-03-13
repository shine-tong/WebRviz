ROS Client Module (rosClient.ts)
================================

Module path: ``web_rviz/src/ros/rosClient.ts``

Purpose
-------

Wraps ``roslib`` connection lifecycle, topic creation, and ``rosapi`` service calls.

Core types
----------

- ``ConnectionState``: ``disconnected | connecting | connected | error``
- ``TopicInfo``: topic name/type pair.
- ``ServiceInfo``: service name/type pair.
- ``MessageTypeDef``: message field metadata.
- ``MessageDetailsResponse``: message details response.
- ``ServiceDetailsResponse``: request/response details response.

RosClient
---------

``onStateChange(listener)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Registers connection state listeners.

``getState()`` / ``isConnected()``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Returns the current state and connection readiness.

``connect(url: string): Promise<void>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Creates a ROS WebSocket connection and handles ``connection``, ``error``, and ``close`` events.

``disconnect(): void``
^^^^^^^^^^^^^^^^^^^^^^

Closes the connection and resets client state.

``createTopic(name, messageType, throttleRateMs = 0)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Creates a ``ROSLIB.Topic`` for subscription/publication.

``listTopicsWithTypes()``
^^^^^^^^^^^^^^^^^^^^^^^^^

Fetches topics with type info via ``/rosapi/topics`` and ``/rosapi/topic_type``.

``listServicesWithTypes()``
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Fetches services with type info via ``/rosapi/services`` and ``/rosapi/service_type``.

``listParams()``
^^^^^^^^^^^^^^^^

Reads parameter names via ``/rosapi/get_param_names``.

``getParam(name)``
^^^^^^^^^^^^^^^^^^

Loads a parameter value string via ``/rosapi/get_param``.

``getMessageDetails(type)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Loads message type definitions via ``/rosapi/message_details``.

``getServiceDetails(type)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Loads request/response details in parallel, with compatibility fallback for ``type`` vs ``service`` key differences.
