cmake_minimum_required(VERSION 2.8 FATAL_ERROR)


set(NAME test_esf)

project(${NAME})

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SRC ../../src/cpr_params.cpp ../../src/cpr_loadfiles.cpp test.cpp)

include_directories("../../includes")

add_executable (${NAME} ${SRC})
target_link_libraries (${NAME} PUBLIC ${PCL_LIBRARIES})
target_compile_options(${NAME} PRIVATE -Wall -Wextra -Wfloat-conversion)
