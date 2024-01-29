cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(race)

find_package(ignition-transport11 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})

include_directories(${CMAKE_BINARY_DIR})

if (EXISTS "${CMAKE_SOURCE_DIR}/race.cc")
    add_executable(race race.cc)
    target_link_libraries(race ignition-transport${IGN_TRANSPORT_VER}::core)
endif()