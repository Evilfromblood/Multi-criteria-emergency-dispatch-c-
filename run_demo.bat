@echo off
setlocal
if exist "C:\msys64\ucrt64\bin\g++.exe" (
    set "PATH=C:\msys64\ucrt64\bin;%PATH%"
)
if not exist "emergency_dispatch.exe" (
    echo emergency_dispatch.exe not found, building first...
    call build.bat
)
emergency_dispatch.exe --demo
