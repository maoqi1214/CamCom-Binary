@echo off
setlocal

if "%~1"=="" (
  echo Usage: build_vs26.bat ^<OpenCV_DIR^>
  echo Example: build_vs26.bat D:\vscode\light\opencv\build\x64\vc16\lib
  exit /b 1
)

set "OPENCV_DIR=%~1"

cmake -S . -B build-vs26 -G "Visual Studio 18 2026" -A x64 -DOpenCV_DIR="%OPENCV_DIR%"
if errorlevel 1 exit /b %errorlevel%

cmake --build build-vs26 --config Debug
exit /b %errorlevel%
