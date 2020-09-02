cmake_minimum_required(VERSION 3.16)
project(IgH_demo)

INCLUDE_DIRECTORIES(/opt/etherlab/include)
LINK_DIRECTORIES(/opt/etherlab/lib)

set(CMAKE_CXX_STANDARD 20)

add_executable(IgH_demo main.cpp)

target_link_libraries(${CMAKE_PROJECT_NAME} ethercat)