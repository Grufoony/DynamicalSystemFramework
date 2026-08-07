cmake_minimum_required(VERSION 3.16.0)

project(
  Benchmark
  VERSION 1.0.1
  LANGUAGES CXX)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Check if being built as part of the main DSF project or standalone
if(NOT TARGET dsf)
    # Standalone build - need to find installed DSF
    find_package(dsf REQUIRED)
endif()

# Get Google Benchmark
include(FetchContent)
set(BENCHMARK_ENABLE_TESTING OFF)
FetchContent_Declare(
    benchmark
    GIT_REPOSITORY https://github.com/google/benchmark
    GIT_TAG v1.9.5
)
FetchContent_MakeAvailable(benchmark)

# add as executable all cpp files into '.' folder
file(GLOB SOURCES "*.cpp")

# Set the exe folder to be the one of the CMakeLists.txt
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

# Loop through each source file and create an executable
foreach(SOURCE ${SOURCES})
    get_filename_component(EXE_NAME ${SOURCE} NAME_WE)
    add_executable(${EXE_NAME}.out ${SOURCE})
    target_compile_definitions(${EXE_NAME}.out PRIVATE SPDLOG_USE_STD_FORMAT)
    target_link_libraries(${EXE_NAME}.out PRIVATE dsf TBB::tbb $<BUILD_INTERFACE:simdjson::simdjson> spdlog::spdlog benchmark::benchmark)
    target_include_directories(${EXE_NAME}.out PRIVATE $<INSTALL_INTERFACE:include>)
endforeach()
