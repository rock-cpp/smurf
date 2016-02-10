find_package( Boost COMPONENTS filesystem)

rock_library(smurf
    SOURCES Robot.cpp 
            Collidable.cpp
            DynamicTransformation.cpp
            Frame.cpp
            Inertial.cpp
            Joint.cpp
            RotationalJoint.cpp
            Sensor.cpp
            StaticTransformation.cpp
            Transformation.cpp
            TranslationalJoint.cpp
    HEADERS Robot.hpp
            Collidable.hpp
            DynamicTransformation.hpp
            Frame.hpp
            Inertial.hpp
            Joint.hpp
            RotationalJoint.hpp
            Sensor.hpp
            StaticTransformation.hpp
            Transformation.hpp
            TranslationalJoint.hpp
            Smurf.hpp
    DEPS_PKGCONFIG
        base-types
        configmaps
        smurf_parser
        base-lib
		mars_interfaces
    DEPS_PLAIN 
        Boost_FILESYSTEM 
)

rock_executable(smurf_bin Main.cpp
    DEPS smurf)

