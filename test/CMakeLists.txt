find_package(Boost)

rock_testsuite(test_suite suite.cpp
    test_robot.cpp
    DEPS_CMAKE Boost
    DEPS_PKGCONFIG orocos_cpp smurf)

target_link_libraries(test_suite ${Boost_LIBRARIES})
