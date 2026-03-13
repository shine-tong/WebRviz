RViz Sync Module (rvizConfig.ts)
================================

Module path: ``web_rviz/src/rviz/rvizConfig.ts``

Purpose
-------

Parses RViz YAML config and extracts fixed-frame and point-cloud topic hints.

Public API
----------

``RvizSyncHints``
^^^^^^^^^^^^^^^^^

- ``fixedFrame?: string``
- ``pointCloudTopics: string[]``

``parseRvizConfig(content: string): RvizSyncHints``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Reads ``Visualization Manager`` fields.
- Extracts ``Fixed Frame`` from ``Global Options``.
- Recursively scans ``Displays`` and collects unique point cloud topics.

``loadRvizSyncHints(url: string): Promise<RvizSyncHints>``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Fetches RViz config from URL.
- Throws on non-2xx responses.
- Parses and returns sync hints.
