#include <boost/test/unit_test.hpp>
#include <smurf/Smurf.hpp>
#include <orocos_cpp/YAMLConfiguration.hpp>

const std::string path = orocos_cpp::YAMLConfigParser::applyStringVariableInsertions("<%=ENV(AUTOPROJ_CURRENT_ROOT) %>/<%=ENV(ASGUARD4)%>");
//const std::string robotPath = orocos_cpp::YAMLConfigParser::applyStringVariableInsertions("<%=ENV(AUTOPROJ_CURRENT_ROOT) %>/<%=ENV(SPACECLIMBER)%>");

BOOST_AUTO_TEST_CASE(test_load_from_smurf)
{
    smurf::Robot robot;
    robot.loadFromSmurf(path);
}

BOOST_AUTO_TEST_CASE(test_get_joints)
{
    smurf::Robot robot;
    robot.loadFromSmurf(path);
    const std::vector<smurf::Joint*> joints = robot.getJoints();
}