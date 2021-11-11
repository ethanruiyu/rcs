cmake_minimum_required(VERSION 3.14)
project(planner LANGUAGES CXX)

# set(CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/GlobalPlanner/lib)
set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
# set(GlobalPlanner_DIR ${PROJECT_SOURCE_DIR}/GlobalPlanner/lib/cmake)
# find_package(GlobalPlanner REQUIRED)

include_directories(
                    ${OpenCV_INCLUDE_DIRS}
                    /usr/local/include/osqp
                    ${EIGEN3_INCLUDE_DIR}
                    ./GlobalPlanner/include)

add_subdirectory(pybind11)


pybind11_add_module(planner_module
  planner.cpp
)

target_link_libraries(planner_module PUBLIC ${PROJECT_SOURCE_DIR}/GlobalPlanner/lib/libopt.a osqp:osqp ${OpenCV_LIBS})