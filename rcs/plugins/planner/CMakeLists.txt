cmake_minimum_required(VERSION 3.14)
project(planner LANGUAGES CXX)

# set(CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/GlobalPlanner/lib)
set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
# set(GlobalPlanner_DIR ${PROJECT_SOURCE_DIR}/GlobalPlanner/lib/cmake)
# find_package(GlobalPlanner REQUIRED)

include_directories(
                    ${OpenCV_INCLUDE_DIRS}
                    ./GlobalPlanner/include)

add_subdirectory(pybind11)


pybind11_add_module(planner_module
  planner.cpp
)

target_link_libraries(planner_module PUBLIC ${OpenCV_LIBS} ${PROJECT_SOURCE_DIR}/GlobalPlanner/lib/libglobal_planner.so)