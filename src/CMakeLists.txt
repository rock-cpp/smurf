rock_library(smurf
    SOURCES Smurf.cpp
    HEADERS Smurf.hpp
    DEPS_PKGCONFIG
        base-types
)

rock_executable(smurf_bin Main.cpp
    DEPS smurf)

