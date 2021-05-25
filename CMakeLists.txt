cmake_minimum_required(VERSION 3.11.3)
project(hybrid_astar_search)

set(CMAKE_CXX_STANDARD 11)

add_executable(hybrid_astar_search main.cpp hybrid_breadth_first.cpp hybrid_breadth_first.h)