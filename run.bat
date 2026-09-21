@echo off
chcp 65001 >nul
setlocal
cd /d "%~dp0"
title 大件运输弯道通过性检验系统

rem ===== 使用本机全局 Python（优先 3.12）=====
set "PY="

rem 1) 优先用 py launcher 选 3.12
py -3.12 -V >nul 2>&1 && set "PY=py -3.12"

rem 2) 退而求其次，直接用 PATH 里的 python
if not defined PY python -V >nul 2>&1 && set "PY=python"

rem 3) 都找不到，报错退出
if not defined PY (
    echo [ERROR] Python 3.12 not found on this machine.
    echo Please install Python 3.12 and make sure "py" works in PATH.
    pause
    exit /b 1
)

rem 健康检查：能打印版本才继续
%PY% -c "import encodings, sys; print(sys.version)" >nul 2>&1
if errorlevel 1 (
    echo [ERROR] The detected Python is broken: %PY%
    echo Check PYTHONHOME / PYTHONPATH, or reinstall Python.
    pause
    exit /b 1
)

rem 依赖自检：核心库缺失时自动调用 pip 安装
%PY% -c "import PyQt5, pydantic, shapely, docx, matplotlib" >nul 2>&1
if errorlevel 1 (
    echo [INFO] Missing dependencies, installing via pip...
    %PY% -m pip install -r requirements.txt
    if errorlevel 1 (
        echo [ERROR] Failed to install dependencies. Please run:
        echo     %PY% -m pip install -r requirements.txt
        pause
        exit /b 1
    )
)

echo Using Python: %PY%
%PY% -u main.py
set "EXITCODE=%ERRORLEVEL%"
echo.
echo Program exited with code %EXITCODE%.
echo If there was an error, the detail is saved in runtime.log in this folder.
pause
exit /b %EXITCODE%