find_package(Boost)

rock_testsuite(test_suite
    suite.cpp
    test_robot.cpp
    DEPS_CMAKE Boost
    DEPS smurf)

target_link_libraries(test_suite ${Boost_LIBRARIES})

add_subdirectory(sample_smurfs)

file(COPY sample_smurfs DESTINATION ./)