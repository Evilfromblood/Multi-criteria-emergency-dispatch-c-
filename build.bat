@echo off
setlocal
if exist "C:\msys64\ucrt64\bin\g++.exe" (
    set "PATH=C:\msys64\ucrt64\bin;%PATH%"
)
echo Compiling V2X Emergency Dispatch (emergency_dispatch.exe)...
g++ -std=c++17 -Wall -Wextra -Iinclude src\road_network.cpp src\router.cpp src\world.cpp src\motion.cpp src\dispatcher.cpp src\simulation.cpp src\telemetry.cpp src\scenario.cpp src\main.cpp -o emergency_dispatch.exe
if %ERRORLEVEL% NEQ 0 (
    echo Build FAILED.
    exit /b 1
)

echo Compiling Tests (dispatch_tests.exe)...
g++ -std=c++17 -Wall -Wextra -Iinclude src\road_network.cpp src\router.cpp src\world.cpp src\motion.cpp src\dispatcher.cpp src\simulation.cpp src\telemetry.cpp src\scenario.cpp tests\test_main.cpp -o dispatch_tests.exe
if %ERRORLEVEL% NEQ 0 (
    echo Test Build FAILED.
    exit /b 1
)

echo.
echo Running unit tests:
dispatch_tests.exe
echo.
echo Build OK. Run: emergency_dispatch.exe --demo (or run_demo.bat)
