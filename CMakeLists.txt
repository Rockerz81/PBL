cmake_minimum_required(VERSION 3.16)
project(LogisticsRouter VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ---- external libraries ----
find_package(CURL REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(raylib 4.0 QUIET)
if(NOT raylib_FOUND)                       # fall back to downloading + building Raylib
  include(FetchContent)
  FetchContent_Declare(raylib GIT_REPOSITORY https://github.com/raysan5/raylib.git GIT_TAG 5.0)
  set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(raylib)
endif()

function(logi_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endfunction()

# ---- layer 1: pure logic, NO external dependencies (unit-testable) ----
add_library(logi_core STATIC src/RoadNetwork.cpp src/DeliveryManager.cpp)
target_include_directories(logi_core PUBLIC include)
logi_warnings(logi_core)

# ---- layer 2: I/O (libcurl + nlohmann/json + fstream) ----
add_library(logi_io STATIC src/OverpassClient.cpp src/Persistence.cpp)
target_link_libraries(logi_io PUBLIC logi_core CURL::libcurl nlohmann_json::nlohmann_json)
logi_warnings(logi_io)

# ---- layer 3: graphical application ----
add_executable(logistics_router src/main.cpp src/MapRenderer.cpp)
target_link_libraries(logistics_router PRIVATE logi_io raylib)
logi_warnings(logistics_router)

# ---- tests ----
enable_testing()
add_executable(core_tests tests/core_test.cpp)
target_link_libraries(core_tests PRIVATE logi_core)
add_test(NAME core_tests COMMAND core_tests)
