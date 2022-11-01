# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 3.3)
find_package(Rock)
set(ROCK_TEST_ENABLED ON)

SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++0x" )

rock_init(smurf 0.1)
rock_standard_layout()
