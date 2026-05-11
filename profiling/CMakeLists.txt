cmake_minimum_required(VERSION 3.16.0)

project(prof VERSION 1.0.1)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(TBB REQUIRED CONFIG)
find_package(spdlog REQUIRED)
find_package(simdjson REQUIRED)

# Set the C++ flags
string(APPEND CMAKE_CXX_FLAGS "-Wall -Wextra -Os")

# Set the folder for the executable
set(EXECUTABLE_OUTPUT_PATH ../)

include(FetchContent)
# Get csv-parser
FetchContent_Declare(
  csv-parser
  GIT_REPOSITORY https://github.com/vincentlaucsb/csv-parser
  GIT_TAG 5.1.0)
FetchContent_GetProperties(csv-parser)
if(NOT csv-parser_POPULATED)
  FetchContent_MakeAvailable(csv-parser)
endif()

file(GLOB SOURCES "../src/dsf/base/*.cpp" "../src/dsf/mobility/*.cpp"
     "../src/dsf/utility/*.cpp" "../src/dsf/geometry/*.cpp")

# Define the executable
add_executable(prof.out main.cpp ${SOURCES})
target_include_directories(prof.out PRIVATE ../src/)
target_link_libraries(prof.out PRIVATE TBB::tbb fmt::fmt spdlog::spdlog
                                       $<BUILD_INTERFACE:simdjson::simdjson> $<BUILD_INTERFACE:csv>)
target_compile_options(prof.out PRIVATE -pg)
target_link_options(prof.out PRIVATE -pg)
add_executable(mem.out main.cpp ${SOURCES})
target_include_directories(mem.out PRIVATE ../src/)
target_link_libraries(mem.out PRIVATE TBB::tbb fmt::fmt spdlog::spdlog
                                      $<BUILD_INTERFACE:simdjson::simdjson> $<BUILD_INTERFACE:csv>)
add_executable(parse_massif.out parse_massif.cpp)
