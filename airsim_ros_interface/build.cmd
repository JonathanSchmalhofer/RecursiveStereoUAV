@echo off
REM //---------- set up variable ----------
setlocal
set ROOT_DIR=%CD%
chdir /d %ROOT_DIR% 

REM //---------- if cmake doesn't exist then install it ----------
WHERE cmake >nul 2>nul
IF %ERRORLEVEL% NEQ 0 (
    call :installcmake
)

REM //---------- compile airsim_ros_interface that we got from git submodule ----------
IF NOT EXIST build mkdir build
cd build
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
%ROOT_DIR%\tools\httpget "https://cmake.org/files/v3.7/cmake-3.7.2-win64-x64.zip"
if ERRORLEVEL 1 goto :cmakefailed
echo Decompressing cmake-3.7.2-win64-x64.zip...
%ROOT_DIR%\tools\unzip "cmake-3.7.2-win64-x64.zip"
if ERRORLEVEL 1 goto :cmakefailed
del cmake-3.7.2-win64-x64.zip
goto :eof_own

:cmakefailed
echo CMake install failed, please install cmake manually from https://cmake.org/
exit 1

:eof_own
pause