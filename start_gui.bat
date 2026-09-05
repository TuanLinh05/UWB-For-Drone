@echo off
setlocal
cd /d "%~dp0GUI test"
echo ========================================
echo   Starting UWB Drone GUI Dev Server
echo ========================================
echo.
if not exist "node_modules" (
    echo node_modules not found. Installing dependencies first...
    call npm install
)
echo Launching GUI...
call npm run dev
pause