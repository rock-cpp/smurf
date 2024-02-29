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
            Motor.cpp
            Material.cpp
            Visual.cpp
    HEADERS Robot.hpp
            Collidable.hpp
            ContactParams.hpp
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
            Motor.hpp
            Material.hpp
            Visual.hpp
    DEPS_PKGCONFIG
        base-types
        configmaps
        smurf_parser
        base-lib
    DEPS
        Boost::filesystem
)

FIND_PACKAGE( Boost COMPONENTS program_options filesystem REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

rock_executable(smurf_dump Main.cpp
    DEPS smurf)
TARGET_LINK_LIBRARIES(smurf_dump ${Boost_LIBRARIES})

