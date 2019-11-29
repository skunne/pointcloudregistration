cmake_minimum_required(VERSION 2.8 FATAL_ERROR)


set(NAME register_pointclouds)

project(${NAME})

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SRC src/main.cpp src/clustering.cpp src/features.cpp src/visualisation.cpp src/params.cpp src/loadfiles.cpp)

include_directories("includes")

add_executable (${NAME} ${SRC})
target_link_libraries (${NAME} PUBLIC ${PCL_LIBRARIES})
target_compile_options(${NAME} PRIVATE -Werror -Wall -Wextra)
