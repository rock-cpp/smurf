rock_library(smurf
    SOURCES Smurf.cpp
    HEADERS Smurf.hpp
    DEPS_PKGCONFIG
        base-types
        configmaps
        smurf_parser
)

rock_executable(smurf_bin Main.cpp
    DEPS smurf)

