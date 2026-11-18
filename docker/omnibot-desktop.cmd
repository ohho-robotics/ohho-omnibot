@echo off
rem OmniBot browser desktop launcher (Windows). Double-click it, or run
rem   docker\omnibot-desktop.cmd [start|stop|restart|logs|shell|status|update|down] [-Port 6081] [-World apartment] [-Password] [-Build]
rem It runs omnibot-desktop.ps1 next to it.
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0omnibot-desktop.ps1" %*
if errorlevel 1 if "%~1"=="" pause
