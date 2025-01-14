cmake_minimum_required(VERSION 3.5)

project(lab4_4 VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5 REQUIRED COMPONENTS Widgets Charts)

add_executable(lab4_4
    main.cpp
    display_window.cpp
    test.cpp
)

target_include_directories(lab4_4 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/DataStructures
)

target_link_libraries(lab4_4 PRIVATE Qt5::Widgets Qt5::Charts)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "Building in Debug mode")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -Wall")
else()
    message(STATUS "Building in Release mode")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
endif()
