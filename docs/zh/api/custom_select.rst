增强下拉框模块（customSelect.ts）
==================================

模块路径：``web_rviz/src/customSelect.ts``

用途
----

在保留原生 ``select`` 作为真实数据源的前提下，为其包上一层可定制样式、支持键盘操作的增强下拉框。

公开接口
--------

``CustomSelectController``
^^^^^^^^^^^^^^^^^^^^^^^^^^

- ``refresh()``：根据当前原生 ``select`` 状态重建自定义选项列表。
- ``dispose()``：移除增强包装，并把原始 ``select`` 原位恢复。

``enhanceSelect(select: HTMLSelectElement): CustomSelectController``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

增强一个 ``select`` 元素并返回其控制器。

``refreshEnhancedSelect(select: HTMLSelectElement): void``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

如果该 ``select`` 已经增强，则刷新对应的自定义菜单内容。

行为说明
--------

- 目标 ``select`` 必须已经挂载到某个父元素下。
- 每个 ``select`` 最多只会创建一个控制器；重复调用 ``enhanceSelect()`` 会复用现有实例。
- 内部使用 ``MutationObserver`` 监听选项变化和禁用状态变化，保证自定义菜单与原生控件同步。
- 点击组件外部区域或按下 ``Escape`` 会关闭所有已展开的增强下拉框。
- 选择新选项时会同步更新原生值，并派发一个冒泡的 ``change`` 事件。
