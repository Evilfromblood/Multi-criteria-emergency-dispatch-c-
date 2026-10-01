cmake_minimum_required(VERSION 3.15)
project(V2XEmergencyDispatch VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_library(dispatch_core STATIC
  src/road_network.cpp
  src/router.cpp
  src/world.cpp
  src/motion.cpp
  src/dispatcher.cpp
  src/simulation.cpp
  src/telemetry.cpp
  src/scenario.cpp)
target_include_directories(dispatch_core PUBLIC include)
if(MSVC)
  target_compile_options(dispatch_core PUBLIC /W4 /utf-8 /EHsc)
else()
  target_compile_options(dispatch_core PUBLIC -Wall -Wextra -Wpedantic)
endif()

add_executable(emergency_dispatch src/main.cpp)
target_link_libraries(emergency_dispatch PRIVATE dispatch_core)

enable_testing()
add_executable(dispatch_tests tests/test_main.cpp)
target_link_libraries(dispatch_tests PRIVATE dispatch_core)
add_test(NAME core_tests COMMAND dispatch_tests)
