UI Settings Module (uiSettings.ts)
==================================

Module path: ``web_rviz/src/uiSettings.ts``

Purpose
-------

Manages persisted language/theme preferences and applies static UI translations.

Public API
----------

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

Union of translation keys derived from the built-in English translation table.

``loadLanguage(): Language`` / ``saveLanguage(language): void``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Loads or stores the current UI language in ``localStorage``.

``loadTheme(): ThemeMode`` / ``saveTheme(theme): void``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Loads or stores the current UI theme in ``localStorage``.

``t(language, key, vars = {})``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Returns the translated string for ``key`` and performs ``{name}`` placeholder substitution from ``vars``.

``applyStaticTranslations(language): void``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Updates ``document.documentElement.lang``, ``document.title``, and every element marked with ``data-i18n``.

``applyTheme(theme): void``
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Writes the current theme to ``document.documentElement.dataset.theme`` for CSS selectors to consume.

Behavior Notes
--------------

- Default language is ``zh`` when no prior preference exists.
- Default theme is ``light`` when no prior preference exists.
- ``t()`` falls back to the English string first, then the raw key.
- ``applyStaticTranslations()`` only updates static labels; dynamic runtime strings should still go through ``t()`` when rendered.
