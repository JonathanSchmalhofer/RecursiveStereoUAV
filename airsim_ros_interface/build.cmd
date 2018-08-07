@echo off
REM //---------- set up variable ----------
setlocal
set ROOT_DIR=%CD%
set MSBUILD_V140_DIR=C:\Program Files (x86)\MSBuild\14.0\Bin
chdir /d %ROOT_DIR% 

REM //---------- if cmake doesn't exist then install it ----------
WHERE cmake >nul 2>nul
IF %ERRORLEVEL% NEQ 0 (
    echo Cmake does not exist yet...
    call :installcmake
) ELSE (
    echo Cmake found.
)
:aftercmakecheck

REM //---------- if BOOST doesn't exist then download it ----------
REM IF "%BOOST_ROOT%"=="" (
REM    ECHO Boost could not be found in BOOST_ROOT, so Boost 1.6.5 will be installed...
REM    call :installboost
REM ) ELSE (
REM    ECHO Boost found in %BOOST_ROOT%
REM )
:afterboostcheck

REM //---------- compile airsim_ros_interface that we got from git submodule ----------
:regular_build_pipeline
echo Starting Build
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

:installboost
if NOT EXIST boost_1_65_0 call :downloadboost
echo Set BOOST_ROOT to path of boost_1_65_0
echo set BOOST_ROOT=%ROOT_DIR%\boost_1_65_0;
set BOOST_ROOT=%ROOT_DIR%\boost_1_65_0;
goto :afterboostecheck

:downloadboost
echo Boost was not found, so we are installing Boost 1.6.5 for you... 
powershell -ExecutionPolicy Unrestricted -file "%ROOT_DIR%\..\tools\http_get_powershell.ps1" -url_download "http://dl.bintray.com/boostorg/release/1.65.0/source/boost_1_65_0.zip"
if ERRORLEVEL 1 goto :boostfailed
echo Decompressing boost_1_65_0.zip...
%ROOT_DIR%\..\tools\unzip "boost_1_65_0.zip"
if ERRORLEVEL 1 goto :boostfailed
del boost_1_65_0.zip

:installcmake
if NOT EXIST cmake-3.7.2-win64-x64 call :downloadcmake
echo Adding cmake-3.7.2-win64-x64 to path
set PATH=%PATH%;%ROOT_DIR%\cmake-3.7.2-win64-x64\bin;
goto :aftercmakecheck

:downloadcmake
echo CMake was not found, so we are installing it for you... 
REM //%ROOT_DIR%\..\tools\HttpGet "https://cmake.org/files/v3.7/cmake-3.7.2-win64-x64.zip"
powershell -ExecutionPolicy Unrestricted -file "%ROOT_DIR%\..\tools\http_get_powershell.ps1" -url_download "https://cmake.org/files/v3.7/cmake-3.7.2-win64-x64.zip"
if ERRORLEVEL 1 goto :cmakefailed
echo Decompressing cmake-3.7.2-win64-x64.zip...
%ROOT_DIR%\..\tools\unzip "cmake-3.7.2-win64-x64.zip"
if ERRORLEVEL 1 goto :cmakefailed
del cmake-3.7.2-win64-x64.zip

:downloadzeromqsrc
cd %ROOT_DIR%\build
if exist "include/zeromq_src/zmq.h" goto :prebuildstep2
echo Downloading ZeroMQ 4.2.2
REM //%ROOT_DIR%\..\tools\HttpGet "https://github.com/zeromq/libzmq/releases/download/v4.2.2/zeromq-4.2.2.tar.gz"
powershell -ExecutionPolicy Unrestricted -file "%ROOT_DIR%\..\tools\http_get_powershell.ps1" -url_download "https://github.com/zeromq/libzmq/releases/download/v4.2.2/zeromq-4.2.2.tar.gz"
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
REM //%ROOT_DIR%\..\tools\HttpGet "https://github.com/zeromq/cppzmq/archive/v4.2.2.tar.gz"
powershell -ExecutionPolicy Unrestricted -file "%ROOT_DIR%\..\tools\http_get_powershell.ps1" -url_download "https://github.com/zeromq/cppzmq/archive/v4.2.2.tar.gz"
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
REM //%ROOT_DIR%\..\tools\HttpGet "https://github.com/google/flatbuffers/archive/v1.8.0.tar.gz"
powershell -ExecutionPolicy Unrestricted -file "%ROOT_DIR%\..\tools\http_get_powershell.ps1" -url_download "https://github.com/google/flatbuffers/archive/v1.8.0.tar.gz"
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
pause
exit 1

:boostfailed
echo Boost install failed, please install Boost 1.6.5 manually from https://www.boost.org/doc/libs/1_65_0/more/getting_started/windows.html
pause
exit 1

:eof_own
pause
exit 0