rock_library(smurf
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(smurf_bin Main.cpp
    DEPS smurf)

