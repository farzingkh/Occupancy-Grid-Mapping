cmake_minimum_required(VERSION 3.17)
add_definitions(-std=c++11)
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(OGM)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
set(CMAKE_BUILD_TYPE Debug)

find_package(PythonLibs 2.7)

include_directories("${PROJECT_SOURCE_DIR}/include/")

# Find all executables
file(GLOB project_SRCS src/*.cpp) #src/*.h

# Add project executable
add_executable(OGM ${project_SRCS})
target_include_directories(OGM PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(OGM ${PYTHON_LIBRARIES})