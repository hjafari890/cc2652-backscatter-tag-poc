@echo off
setlocal
set PYTHON_EXE=C:\Users\hjafa\radioconda\python.exe
set SCRIPT_DIR=%~dp0dashboard
"%PYTHON_EXE%" "%SCRIPT_DIR%\backscatter_poc_dashboard.py" %*
endlocal
