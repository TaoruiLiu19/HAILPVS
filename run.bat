@echo off
setlocal EnableExtensions
title Oversize Transport Passability Check
cd /d "%~dp0"

rem --- pick a Python interpreter, most specific first ---
set "PY="

rem 1) primary: full path of the global interpreter (edit PYPRI to taste)
set "PYPRI=E:\python\Python312\python.exe"
if exist "%PYPRI%" (
    set "PY=%PYPRI%"
    goto :found
)

rem 2) fallback: py launcher, only if it can actually see 3.12
where py >nul 2>&1
if not errorlevel 1 (
    py -0p 2>nul | findstr /i /c:"3.12" >nul
    if not errorlevel 1 (
        set "PY=py -3.12"
        goto :found
    )
)

rem 3) fallback: bare python on PATH
where python >nul 2>&1
if not errorlevel 1 (
    set "PY=python"
    goto :found
)

echo [ERROR] No suitable Python runtime found.
echo Please install Python 3.12, or edit PYPRI near the top of run.bat
echo to point at your full python.exe path.
pause
exit /b 1

:found
echo [INFO] Using: %PY%
%PY% -V

rem --- dependency self-check; only install when something is missing ---
%PY% -c "import PyQt5, pydantic, shapely, docx, matplotlib" >nul 2>&1
if errorlevel 1 (
    echo [INFO] Missing dependencies, installing via pip...
    %PY% -m pip install -r requirements.txt
    if errorlevel 1 (
        echo [ERROR] Failed to install dependencies.
        echo Please run manually:  %PY% -m pip install -r requirements.txt
        pause
        exit /b 1
    )
)

echo [INFO] Launching main.py ...
%PY% -u main.py
pause