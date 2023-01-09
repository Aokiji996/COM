cmake_minimum_required(VERSION 3.22)
project(COM VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 14)

add_executable(COM main.cpp com.cpp com.h test.cpp)
find_package(Threads REQUIRED)
target_link_libraries(COM Threads::Threads)
