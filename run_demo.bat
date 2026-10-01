@echo off
setlocal
if exist "C:\msys64\ucrt64\bin\g++.exe" (
    set "PATH=C:\msys64\ucrt64\bin;%PATH%"
)
if not exist "dispatch_demo.exe" (
    echo dispatch_demo.exe not found, building first...
    call build.bat
)
dispatch_demo.exe --demo
