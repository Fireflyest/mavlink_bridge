cmake_minimum_required(VERSION 3.10)
project(mavlink_bridge C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(MAVSDK REQUIRED)

add_executable(mavlink_bridge
    src/main.cpp
    src/bridge.cpp
    src/vla_handler.cpp
    src/console_handler.cpp
)

target_include_directories(mavlink_bridge PRIVATE include)
target_link_libraries(mavlink_bridge MAVSDK::mavsdk pthread)
