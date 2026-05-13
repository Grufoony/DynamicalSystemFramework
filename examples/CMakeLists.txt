cmake_minimum_required(VERSION 3.16.0)

project(
  dsf_examples
  VERSION 3.0.0
  LANGUAGES CXX)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add -pg flag to enable profiling if DSF_PROFILE is ON
if(DSF_PROFILE)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
  endif()
endif()

# Check if being built as part of the main DSF project or standalone
if(NOT TARGET dsf)
  # Standalone build - need to find installed DSF
  find_package(dsf REQUIRED)
endif()

# add as executable all cpp files into '.' folder
file(GLOB SOURCES "*.cpp")

# Set the exe folder to be the one of the CMakeLists.txt
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

# Loop through each source file and create an executable
foreach(SOURCE ${SOURCES})
  get_filename_component(EXE_NAME ${SOURCE} NAME_WE)
  add_executable(${EXE_NAME}.out ${SOURCE})
  target_compile_definitions(${EXE_NAME}.out PRIVATE SPDLOG_USE_STD_FORMAT)
  target_link_libraries(${EXE_NAME}.out PRIVATE dsf TBB::tbb simdjson::simdjson
                                                spdlog::spdlog)
  target_include_directories(${EXE_NAME}.out
                             PRIVATE $<INSTALL_INTERFACE:include>)
endforeach()
