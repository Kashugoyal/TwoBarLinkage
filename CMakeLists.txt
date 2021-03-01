cmake_minimum_required(VERSION 3.0.0)
project(TwoBarLinkage VERSION 0.1.0)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -ggdb")
set(CMAKE_CXX_STANDARD 11)

include_directories(./include)
file(
    GLOB
    source_files
    src/*.cpp
)

file(
    GLOB
    test_files
    src/two_bar.cpp 
    src/path_planner.cpp
    test/*.cpp
)

add_executable(TwoBarLinkage ${source_files})

# Download and unpack googletest at configure time
configure_file(CMakeLists.txt.googletest googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/googletest-src
                 ${CMAKE_CURRENT_BINARY_DIR}/googletest-build
                 EXCLUDE_FROM_ALL)


add_executable(tests ${test_files})
target_link_libraries(tests gtest_main)