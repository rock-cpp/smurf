#include <boost/test/unit_test.hpp>
#include <smurf/Smurf.hpp>
#include <orocos_cpp/YAMLConfiguration.hpp>
#include <urdf_model/link.h>
using namespace std;

const string path = orocos_cpp::YAMLConfigParser::applyStringVariableInsertions("<%=ENV(AUTOPROJ_CURRENT_ROOT) %>/<%=ENV(ASGUARD4)%>");
//const string robotPath = orocos_cpp::YAMLConfigParser::applyStringVariableInsertions("<%=ENV(AUTOPROJ_CURRENT_ROOT) %>/<%=ENV(SPACECLIMBER)%>");

BOOST_AUTO_TEST_CASE(test_load_from_smurf)
{
    smurf::Robot robot;
    robot.loadFromSmurf(path);
}

BOOST_AUTO_TEST_CASE(test_get_joints)
{
    smurf::Robot robot;
    robot.loadFromSmurf(path);
    const vector<smurf::Joint*> joints = robot.getJoints();
}

BOOST_AUTO_TEST_CASE(test_get_visuals)
{
    smurf::Robot robot;
    robot.loadFromSmurf(path);
    
    for(const smurf::Frame* pFrame : robot.getFrames())
    {
        vector<urdf::Visual> visuals;
        pFrame->getVisuals(visuals);
        for(const urdf::Visual& v : visuals)
        {
            switch(v.geometry->type)
            {
              case urdf::Geometry::SPHERE:
               // std::cout << "SPERE" << std::endl;
                break;
              case urdf::Geometry::BOX:
             //  std::cout << "BOX" << std::endl;
                break;
              case urdf::Geometry::CYLINDER:
             //   std::cout << "CYLINDER" << std::endl;
                break;
              case urdf::Geometry::MESH:
              //  std::cout << "MESH" << std::endl;
                urdf::Mesh* mesh = dynamic_cast<urdf::Mesh*>(v.geometry.get());
                cout << mesh->filename <<  endl;
                break;
            }
        }
    }
}


