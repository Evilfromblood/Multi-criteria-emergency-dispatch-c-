@echo off
REM Build + test with Visual Studio (MSVC) and CMake.
REM Run from a "Developer Command Prompt for VS" (or any shell where cmake is on PATH).
setlocal
cmake -S . -B build || goto :error
cmake --build build --config Release || goto :error
pushd build
ctest -C Release --output-on-failure || (popd & goto :error)
popd
echo.
echo Build OK.  Run:  build\Release\emergency_dispatch.exe   (or run_demo.bat)
goto :eof
:error
echo Build FAILED.
exit /b 1
