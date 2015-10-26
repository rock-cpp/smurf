find_package( Boost COMPONENTS filesystem)

rock_library(smurf
    SOURCES Smurf.cpp
    HEADERS Smurf.hpp
    DEPS_PKGCONFIG
        base-types
        configmaps
        smurf_parser
    DEPS_PLAIN 
        Boost_FILESYSTEM 
)

rock_executable(smurf_bin Main.cpp
    DEPS smurf)

