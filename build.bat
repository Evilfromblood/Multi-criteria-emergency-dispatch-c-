@echo off
setlocal

REM ── Compiler detection (tries MinGW locations in order of preference) ────────
if exist "C:\msys64\ucrt64\bin\g++.exe" (
    set "PATH=C:\msys64\ucrt64\bin;%PATH%"
    echo [build] Using MSYS2 UCRT64 at C:\msys64\ucrt64\bin  (GCC 16, full C++17)
) else if exist "C:\msys64\mingw64\bin\g++.exe" (
    set "PATH=C:\msys64\mingw64\bin;%PATH%"
    echo [build] Using MSYS2 MinGW64 at C:\msys64\mingw64\bin
) else if exist "C:\MinGW\bin\g++.exe" (
    set "PATH=C:\MinGW\bin;%PATH%"
    echo [build] Using legacy MinGW at C:\MinGW\bin  ^(GCC 6 - may lack C++17^)
) else (
    echo [build] Using g++ from system PATH (no local MinGW found)
)

REM Shared source files (add new .cpp files here)
set SRCS=src\road_network.cpp src\router.cpp src\world.cpp src\motion.cpp ^
         src\dispatcher.cpp src\simulation.cpp src\telemetry.cpp ^
         src\scenario.cpp src\routevis.cpp

echo.
echo Compiling V2X Emergency Dispatch (emergency_dispatch.exe)...
g++ -std=c++17 -Wall -Wextra -Iinclude %SRCS% src\main.cpp -o emergency_dispatch.exe
if %ERRORLEVEL% NEQ 0 (
    echo Build FAILED.
    exit /b 1
)

echo.
echo Compiling Tests (dispatch_tests.exe)...
g++ -std=c++17 -Wall -Wextra -Iinclude %SRCS% tests\test_main.cpp -o dispatch_tests.exe
if %ERRORLEVEL% NEQ 0 (
    echo Test Build FAILED.
    exit /b 1
)

echo.
echo Running unit tests:
dispatch_tests.exe
echo.
echo Build OK.
echo   Run simulation  : emergency_dispatch.exe
echo   Run demo        : emergency_dispatch.exe --demo
echo   Route visualizer: emergency_dispatch.exe --route
echo   (or use run_demo.bat)

