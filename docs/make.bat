@ECHO OFF

set SPHINXBUILD=sphinx-build
if "%1"=="" goto help

if "%1"=="html-zh" (
  set DOC_LANG=zh
  %SPHINXBUILD% -b html -c . zh _build\html-zh -W --keep-going
  goto end
)

if "%1"=="html-en" (
  set DOC_LANG=en
  %SPHINXBUILD% -b html -c . en _build\html-en -W --keep-going
  goto end
)

if "%1"=="html-all" (
  call %0 html-zh
  if errorlevel 1 goto end
  call %0 html-en
  if errorlevel 1 goto end

  if exist _build\site rmdir /s /q _build\site
  mkdir _build\site\en
  xcopy _build\html-zh\* _build\site\ /E /I /Y >nul
  xcopy _build\html-en\* _build\site\en\ /E /I /Y >nul
  type nul > _build\site\.nojekyll
  goto end
)

if "%1"=="clean" (
  if exist _build rmdir /s /q _build
  goto end
)

:help
echo.
echo Please use one of the following targets:
echo   make.bat html-zh
echo   make.bat html-en
echo   make.bat html-all
echo   make.bat clean
exit /b 1

:end
