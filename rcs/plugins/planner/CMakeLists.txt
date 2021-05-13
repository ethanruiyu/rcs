cmake_minimum_required(VERSION 3.5.1)
project(fleet_planner)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
include_directories( include
                    ${OpenCV_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    )

set(CMAKE_CXX_STANDARD 14)

add_subdirectory(pybind11)

pybind11_add_module(planner_module
        src/grass_path_planner.cpp


        src/CoveragePathPlanner/normal_astar_planner/DubinsCurve.cpp
        src/CoveragePathPlanner/normal_astar_planner/node3d.cpp
        src/CoveragePathPlanner/normal_astar_planner/normal_astar_planner_core.cpp

        src/CoveragePathPlanner/prm_planner/graph.cpp
        src/CoveragePathPlanner/prm_planner/prm_planner.cpp

        src/CoveragePathPlanner/cgutil.cpp
        src/CoveragePathPlanner/CoveragePathPlannerCore.cpp
        src/CoveragePathPlanner/CPP_BCD.cpp
        src/CoveragePathPlanner/map_rotate.cpp
        src/CoveragePathPlanner/path_planner.cpp
        src/CoveragePathPlanner/ser_start_end.cpp
        src/CoveragePathPlanner/torres_etal.cpp
        src/CoveragePathPlanner/WeltPathPlanner.cpp

        src/DiffGlobalPlanner/DiffGlobalPathPlanner.cpp
    )
target_link_libraries(planner_module PUBLIC ${OpenCV_LIBS})