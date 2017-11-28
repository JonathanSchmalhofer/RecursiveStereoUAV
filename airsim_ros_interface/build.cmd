@echo off
REM //---------- set up variable ----------
setlocal
set ROOT_DIR=%CD%
set MSBUILD_V140_DIR=C:\Program Files (x86)\MSBuild\14.0\Bin
chdir /d %ROOT_DIR% 

REM //---------- if cmake doesn't exist then install it ----------
WHERE cmake >nul 2>nul
IF %ERRORLEVEL% NEQ 0 (
    call :installcmake
)

REM //---------- compile airsim_ros_interface that we got from git submodule ----------
IF NOT EXIST build mkdir build
cd build
goto :downloadzeromqsrc
:prebuildstep1
goto :installzeromqsrc
:prebuildstep2
goto :downloadzeromqcpp
:prebuildstep3
goto :installzeromqcpp
:prebuildstep4
goto :downloadflatbufferssrc
:prebuildstep5
goto :installflatbufferssrc
:prebuildstep6
cmake -G"Visual Studio 14 2015 Win64" ..
cmake --build . --config Debug
REM // cmake --build . --config Release
if ERRORLEVEL 1 goto :buildfailed
chdir /d %ROOT_DIR% 

REM //---------- done building ----------
goto :eof_own

:buildfailed
chdir /d %ROOT_DIR% 
echo #### Build failed
goto :eof_own

:installcmake
if NOT EXIST cmake-3.7.2-win64-x64 call :downloadcmake
set PATH=%PATH%;%ROOT_DIR%\cmake-3.7.2-win64-x64\bin;
goto :eof_own

:downloadcmake
echo CMake was not found, so we are installing it for you... 
%ROOT_DIR%\..\tools\httpget "https://cmake.org/files/v3.7/cmake-3.7.2-win64-x64.zip"
if ERRORLEVEL 1 goto :cmakefailed
echo Decompressing cmake-3.7.2-win64-x64.zip...
%ROOT_DIR%\tools\unzip "cmake-3.7.2-win64-x64.zip"
if ERRORLEVEL 1 goto :cmakefailed
del cmake-3.7.2-win64-x64.zip
goto :eof_own

:downloadzeromqsrc
cd %ROOT_DIR%\build
if exist "include/zeromq_src/zmq.h" goto :prebuildstep2
echo Downloading ZeroMQ 4.2.2
%ROOT_DIR%\..\tools\httpget "https://github.com/zeromq/libzmq/releases/download/v4.2.2/zeromq-4.2.2.tar.gz"
if ERRORLEVEL 1 goto :buildfailed
echo Decompressing zeromq-4.2.2.tar.gz
%ROOT_DIR%\..\tools\TarTool "zeromq-4.2.2.tar.gz" ./
if ERRORLEVEL 1 goto :buildfailed
del zeromq-4.2.2.tar.gz
goto :prebuildstep1

:installzeromqsrc
cd %ROOT_DIR%\build
if not exist "include" mkdir include && cd include && if not exist "zeromq_src" mkdir zeromq_src
cd %ROOT_DIR%\build\zeromq-4.2.2\builds\msvc\vs2015\libzmq && "%MSBUILD_V140_DIR%\MSBuild" libzmq.vcxproj /property:Configuration=DebugDLL /p:Platform=x64
cd %ROOT_DIR%\build
if not exist "output" mkdir output && cd output && if not exist "bin" mkdir bin && cd bin && if not exist "debug" mkdir debug
cd %ROOT_DIR%\build
if not exist "output" mkdir output && cd output && if not exist "lib" mkdir lib && cd lib && if not exist "debug" mkdir debug
cd %ROOT_DIR%\build
robocopy "zeromq-4.2.2/include" "include/zeromq_src" "zmq.h" "zmq_utils.h"
robocopy "zeromq-4.2.2/bin/x64/Debug/v140/dynamic" "output/lib/Debug"
robocopy "zeromq-4.2.2/bin/x64/Debug/v140/dynamic" "output/bin/Debug" "libzmq.dll"
goto :prebuildstep2

:downloadzeromqcpp
cd %ROOT_DIR%\build
if exist "include/zeromq_cpp/zmq.hpp" goto :prebuildstep4
echo Downloading ZeroMQ CPP 4.2.2
%ROOT_DIR%\..\tools\httpget "https://github.com/zeromq/cppzmq/archive/v4.2.2.tar.gz"
if ERRORLEVEL 1 goto :buildfailed
echo Decompressing v4.2.2.tar.gz
%ROOT_DIR%\..\tools\TarTool "v4.2.2.tar.gz" ./
if ERRORLEVEL 1 goto :buildfailed
del v4.2.2.tar.gz
goto :prebuildstep3

:installzeromqcpp
cd %ROOT_DIR%\build
if not exist "include" mkdir include && cd include && if not exist "zeromq_cpp" mkdir zeromq_cpp
cd %ROOT_DIR%\build
robocopy "cppzmq-4.2.2" "include/zeromq_cpp" "zmq.hpp"
goto :prebuildstep4

:downloadflatbufferssrc
cd %ROOT_DIR%\build
if exist "include/flatbuffers_src/flatbuffers/base.h" goto :prebuildstep6
echo Downloading Flatbuffers 1.8.0
%ROOT_DIR%\..\tools\httpget "https://github.com/google/flatbuffers/archive/v1.8.0.tar.gz"
if ERRORLEVEL 1 goto :buildfailed
echo Decompressing v1.8.0.tar.gz
%ROOT_DIR%\..\tools\TarTool "v1.8.0.tar.gz" ./
if ERRORLEVEL 1 goto :buildfailed
del v1.8.0.tar.gz
goto :prebuildstep5

:installflatbufferssrc
cd %ROOT_DIR%\build
if not exist "include" mkdir include && cd include && if not exist "flatbuffers_src" mkdir flatbuffers_src && cd flatbuffers_src && if not exist "flatbuffers" mkdir flatbuffers 
cd %ROOT_DIR%\build
robocopy "flatbuffers-1.8.0/include/flatbuffers" "include/flatbuffers_src/flatbuffers"
goto :prebuildstep6


:cmakefailed
echo CMake install failed, please install cmake manually from https://cmake.org/
exit 1

:eof_own
pause
exit 0