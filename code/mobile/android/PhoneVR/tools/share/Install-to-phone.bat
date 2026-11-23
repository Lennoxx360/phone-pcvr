@echo off
rem Installs the Phone PCVR APK on a USB-connected Android phone using adb.
rem If you don't have adb, just copy the .apk to your phone and open it there instead.
setlocal
cd /d "%~dp0"
set APK=__APK__

where adb >nul 2>nul
if errorlevel 1 (
  if exist "%LOCALAPPDATA%\Android\Sdk\platform-tools\adb.exe" (
    set "PATH=%LOCALAPPDATA%\Android\Sdk\platform-tools;%PATH%"
  ) else if exist "%~dp0platform-tools\adb.exe" (
    set "PATH=%~dp0platform-tools;%PATH%"
  ) else (
    echo adb was not found.
    echo.
    echo Easiest fix: copy %APK% to your phone ^(USB, Google Drive, Discord...^)
    echo and tap it there. See HOW-TO-INSTALL.txt.
    echo.
    echo Or download Google's "SDK Platform-Tools for Windows", unzip it so the
    echo platform-tools folder sits next to this file, and run this again.
    pause
    exit /b 1
  )
)

echo Waiting for the phone... ^(USB debugging must be on; accept the prompt on the phone^)
adb wait-for-device
echo Installing %APK% ...
adb install -r "%APK%"
if errorlevel 1 (
  echo.
  echo Install failed. If it says INSTALL_FAILED_UPDATE_INCOMPATIBLE, uninstall
  echo the old Phone PCVR from the phone first, then run this again.
  pause
  exit /b 1
)
echo.
echo Installed. Open "Phone PCVR" on your phone.
pause
