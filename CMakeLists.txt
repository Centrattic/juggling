cmake_minimum_required(VERSION 3.16)
project(juggling)

find_package(drake CONFIG REQUIRED)

add_executable(juggling juggling.cc arm_with_cup.cc)
target_link_libraries(juggling drake::drake)
