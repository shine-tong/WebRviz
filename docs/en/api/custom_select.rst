Custom Select Module (customSelect.ts)
======================================

Module path: ``web_rviz/src/customSelect.ts``

Purpose
-------

Wraps native ``select`` elements with a styled, keyboard-accessible custom dropdown while keeping the native control as the source of truth.

Public API
----------

``CustomSelectController``
^^^^^^^^^^^^^^^^^^^^^^^^^^

- ``refresh()``: rebuilds the custom option list from the current native ``select`` state.
- ``dispose()``: removes enhancement markup and restores the original ``select`` in place.

``enhanceSelect(select: HTMLSelectElement): CustomSelectController``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Enhances one ``select`` element and returns its controller.

``refreshEnhancedSelect(select: HTMLSelectElement): void``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Refreshes an already-enhanced ``select`` if a controller exists.

Behavior Notes
--------------

- The target ``select`` must already be attached to a parent element.
- Each ``select`` gets at most one controller; repeated ``enhanceSelect()`` calls reuse the existing instance.
- A ``MutationObserver`` watches option and disabled-state changes so the custom menu stays in sync.
- Clicking outside the control or pressing ``Escape`` closes all open custom selects.
- Selecting a new option updates the native value and dispatches a bubbling ``change`` event.
