@echo off

SET GECKO_OS_SDK_PATH=C:\Users\samia\gecko_os_workspace\sdk-wgm160p\4.0.18-2139

IF NOT EXIST "%GECKO_OS_SDK_PATH%" (
ECHO .
ECHO --------------------------------------------
ECHO ERROR: Gecko OS SDK NOT FOUND
ECHO --------------------------------------------
ECHO The Gecko OS SDK directory was not found at "%GECKO_OS_SDK_PATH%".
ECHO Use Gecko Software Suite, a.k.a. GSS, to updates this project's settings.
ECHO .
EXIT /B
)

set "PROJECT_DIRECTORY=%~dp0%"
for %%i in ("%~dp0..") do set "PROJECT_PARENT_DIRECTORY=%%~fi"

for %%a in ("%~dp0\.") do set "PROJECT_DIRECTORY_NAME=%%~nxa"
pushd "%~dp0"

cd /d "%GECKO_OS_SDK_PATH%"

call .\make.bat ECLIPSE_BUILD=1 PROJECT_PARENT_DIRECTORY.%PROJECT_DIRECTORY_NAME% %* 2>&1 | .\tools\common\win32\tee.exe "%PROJECT_DIRECTORY%build.log"
%SYSTEMROOT%\System32\find "Error 2" "%PROJECT_DIRECTORY%build.log" >nul
IF %errorlevel% equ 1 GOTO finished

:report_build_error
::call

:finished
popd