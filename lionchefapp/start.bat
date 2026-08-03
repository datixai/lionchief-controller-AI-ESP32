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

:: Try venv in lionchefapp folder first, then parent folder
call .venv\Scripts\activate.bat 2>nul
if not errorlevel 1 goto :venv_ok

call ..\.venv\Scripts\activate.bat 2>nul
if not errorlevel 1 goto :venv_ok

call ..\venv\Scripts\activate.bat 2>nul
if not errorlevel 1 goto :venv_ok

call ..\env\Scripts\activate.bat 2>nul
if not errorlevel 1 goto :venv_ok

:: None found
echo [ERROR] Virtual environment not found.
echo.
echo Run this in terminal then try again:
echo.
echo   cd C:\Users\peter\Documents\Linchief_train
echo   .venv\Scripts\activate
echo   cd lionchefapp
echo   pip install flask flask-socketio
echo.
pause
exit /b 1

:venv_ok
echo  [OK] Virtual environment activated.

:: Install flask silently in case not installed yet
pip install flask flask-socketio --quiet 2>nul

:: Open browser after 3 seconds
start /b cmd /c "timeout /t 3 /nobreak >nul && start http://localhost:5000"

:: Run web server
echo  Opening browser at http://localhost:5000
echo  Press Ctrl+C to stop.
echo.
python web_app.py

pause