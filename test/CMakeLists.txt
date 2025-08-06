cmake_minimum_required(VERSION 3.28.3)

project(UnitTests)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/.." "LibBuild")

add_executable(UnitTests "${CMAKE_CURRENT_SOURCE_DIR}/test.cpp")

target_link_libraries(UnitTests PRIVATE DataHandler)
