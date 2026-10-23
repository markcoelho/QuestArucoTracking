@echo off
rem Builds libarucobridge.so (the native ArUco plugin) for Quest (arm64-v8a).
rem Only needed if you change arucobridge.cpp: the built library is already in
rem Assets\Plugins\Android\arm64-v8a.
rem
rem Requirements:
rem   - Unity 6000.3.26f1 with Android Build Support (provides the NDK)
rem   - The OpenCV 5 Android SDK (only its C++ headers are used)
rem
rem Override the defaults by setting these before running:
rem   set UNITY_NDK=C:\path\to\NDK
rem   set OPENCV_SDK=C:\path\to\OpenCV-android-sdk

setlocal
set "HERE=%~dp0"
if not defined UNITY_NDK set "UNITY_NDK=C:\Program Files\Unity\Hub\Editor\6000.3.26f1\Editor\Data\PlaybackEngines\AndroidPlayer\NDK"
if not defined OPENCV_SDK set "OPENCV_SDK=%HERE%..\..\OpenCV-android-sdk"

set "CLANG=%UNITY_NDK%\toolchains\llvm\prebuilt\windows-x86_64\bin\clang++.exe"
set "PLUGINS=%HERE%..\Assets\Plugins\Android\arm64-v8a"

if not exist "%CLANG%" (
  echo NDK clang not found: "%CLANG%"
  exit /b 1
)
if not exist "%OPENCV_SDK%\sdk\native\jni\include\opencv2\core.hpp" (
  echo OpenCV headers not found under "%OPENCV_SDK%\sdk\native\jni\include"
  exit /b 1
)

"%CLANG%" --target=aarch64-linux-android32 -std=c++17 -O3 -fPIC -shared ^
  -fvisibility=hidden -ffunction-sections -fdata-sections ^
  -I"%OPENCV_SDK%\sdk\native\jni\include" ^
  "%HERE%arucobridge.cpp" ^
  -L"%PLUGINS%" -lopencv_java5 ^
  -Wl,--gc-sections -Wl,--build-id=sha1 -Wl,-z,max-page-size=16384 -s ^
  -o "%PLUGINS%\libarucobridge.so"
if errorlevel 1 exit /b 1

echo Built "%PLUGINS%\libarucobridge.so"
