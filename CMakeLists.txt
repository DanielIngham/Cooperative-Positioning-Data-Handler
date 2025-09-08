cmake_minimum_required(VERSION 3.28.3)

project(DataHandler)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

file(GLOB_RECURSE SOURCE_FILES CONFIGURE_DEPENDS 
	"${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/external/gnuplot")

# Create compile commands for LSP
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(DataHandler STATIC ${SOURCE_FILES})

target_include_directories(DataHandler PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

target_compile_definitions(DataHandler PRIVATE
	LIB_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
	PROJECT_DIR="${CMAKE_SOURCE_DIR}"
)

target_link_libraries(DataHandler PUBLIC gnuplot_iostream)
