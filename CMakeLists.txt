cmake_minimum_required(VERSION 3.16)
project(juggling_demo)

find_package(drake CONFIG REQUIRED)

add_executable(juggling_demo juggling_demo.cc)
target_link_libraries(juggling_demo drake::drake)
