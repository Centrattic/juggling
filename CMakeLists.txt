cmake_minimum_required(VERSION 3.16)
project(test_drake)

# Find the Drake package that was installed by apt
find_package(drake CONFIG REQUIRED)

add_executable(test_drake test_drake.cc)

# Link against the monolithic drake library target
target_link_libraries(test_drake drake::drake)

