@echo off
rem Double-click to start Botswain (no console window, no debugger). Errors go to logs\botswain.log.
cd /d "%~dp0"
start "" ".venv\Scripts\pythonw.exe" run_botswain.py %*
