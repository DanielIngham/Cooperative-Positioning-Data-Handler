cmake_minimum_required(VERSION 3.28.3)

project(DataHandler)

# Set the c++ version to 17 for std::filesystem.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB_RECURSE SOURCE_FILES CONFIGURE_DEPENDS "src/*.cpp")

# Create compile commands for LSP
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(DataHandler STATIC ${SOURCE_FILES})

target_include_directories(DataHandler PUBLIC 
	"${CMAKE_CURRENT_SOURCE_DIR}"
)

target_compile_definitions(DataHandler PRIVATE
	LIB_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
	PROJECT_DIR="${CMAKE_SOURCE_DIR}"
)

