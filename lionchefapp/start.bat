@echo off
title LionChief Web Control
echo.
echo  +--------------------------------------------------+
echo  ^|  LionChief Web Interface  ^|  Datix AI  ^|  2026  ^|
echo  +--------------------------------------------------+
echo  ^|  Starting...                                     ^|
echo  +--------------------------------------------------+
echo.

cd /d "%~dp0"

:: Activate virtual environment
call .venv\Scripts\activate.bat 2>nul
if errorlevel 1 (
    echo [ERROR] Virtual environment not found.
    echo Run setup first:
    echo   python -m venv .venv
    echo   .venv\Scripts\activate
    echo   pip install flask flask-socketio opencv-contrib-python bleak numpy
    pause
    exit /b 1
)

:: Install/check flask (silent)
pip install flask flask-socketio --quiet 2>nul

:: Open browser after 3 seconds
start /b cmd /c "timeout /t 3 /nobreak >nul && start http://localhost:5000"

:: Run web server
echo  Opening browser at http://localhost:5000
echo  Press Ctrl+C to stop.
echo.
python web_app.py

pause
