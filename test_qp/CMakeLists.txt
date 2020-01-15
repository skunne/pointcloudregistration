cmake_minimum_required(VERSION 2.8 FATAL_ERROR)


set(NAME test_qp)

project(${NAME})

find_package(PCL 1.8 REQUIRED)
find_package(CGAL COMPONENTS Core)
find_package(GLPK REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SRC main_test.cpp ../src/cpr_processedpointcloud.cpp ../src/cpr_clustering.cpp ../src/cpr_connectedcomponents.cpp ../src/cpr_features.cpp ../src/cpr_matrices.cpp ../src/cpr_graphmatching.cpp ../src/cpr_graphmatching_cgal.cpp ../src/cpr_graphmatching_path.cpp ../src/cpr_visualisation.cpp ../src/cpr_params.cpp ../src/cpr_loadfiles.cpp)

include_directories("../includes")

add_executable (${NAME} ${SRC})
target_link_libraries (${NAME} PUBLIC ${PCL_LIBRARIES})
target_link_libraries (${NAME} PUBLIC glpk)
target_compile_options(${NAME} PRIVATE -Wall -Wextra -Wfloat-conversion)
