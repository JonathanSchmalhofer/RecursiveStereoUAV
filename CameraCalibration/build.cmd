@echo off
REM //---------- set up variable ----------
setlocal
set ROOT_DIR=%CD%
chdir /d %ROOT_DIR% 

:downloadcameracalibration
chdir /d %ROOT_DIR% 
IF NOT EXIST Environments\CameraCalibration\Blocks (
    echo CameraCalibration Environment was not found, so we are downloading it for you... 
    call :downloadcameracalibration
    goto :eof_own
) else (
    echo CameraCalibration Environment found.
)

:downloadcameracalibration
powershell -ExecutionPolicy Unrestricted -file "%ROOT_DIR%\..\tools\http_get_powershell.ps1" -url_download "https://github.com/JonathanSchmalhofer/RecursiveStereoUAV/releases/download/v0.1/Environments.zip"
if ERRORLEVEL 1 goto :cameracalibrationfailed
echo Decompressing Environments.zip...
%ROOT_DIR%\..\tools\unzip "Environments.zip"
del Environments.zip

:cameracalibrationfailed
echo Camera Calibration Installation failed, please install Camera Calibration Environment manually from https://github.com/JonathanSchmalhofer/RecursiveStereoUAV
pause
exit 1

:eof_own
pause
exit 0