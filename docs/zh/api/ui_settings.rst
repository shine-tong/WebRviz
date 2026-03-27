界面设置模块（uiSettings.ts）
=============================

模块路径：``web_rviz/src/uiSettings.ts``

用途
----

管理界面语言与主题的持久化偏好，并负责应用静态文案翻译。

公开接口
--------

``Language``
^^^^^^^^^^^^

- ``en``
- ``zh``

``ThemeMode``
^^^^^^^^^^^^^

- ``dark``
- ``light``

``TranslationKey``
^^^^^^^^^^^^^^^^^^

由内置英文翻译表推导出的合法翻译键联合类型。

``loadLanguage(): Language`` / ``saveLanguage(language): void``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

从 ``localStorage`` 读取或保存当前界面语言。

``loadTheme(): ThemeMode`` / ``saveTheme(theme): void``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

从 ``localStorage`` 读取或保存当前界面主题。

``t(language, key, vars = {})``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

返回 ``key`` 对应的翻译文本，并使用 ``vars`` 对 ``{name}`` 形式的占位符做替换。

``applyStaticTranslations(language): void``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

更新 ``document.documentElement.lang``、``document.title``，以及所有带有 ``data-i18n`` 的元素文本。

``applyTheme(theme): void``
^^^^^^^^^^^^^^^^^^^^^^^^^^^

把当前主题写入 ``document.documentElement.dataset.theme``，供 CSS 主题选择器使用。

行为说明
--------

- 未保存历史偏好时，默认语言为 ``zh``。
- 未保存历史偏好时，默认主题为 ``light``。
- ``t()`` 会优先回退到英文翻译，其次回退到原始 key。
- ``applyStaticTranslations()`` 只负责静态标签；运行时动态文本仍应在渲染时显式调用 ``t()``。
