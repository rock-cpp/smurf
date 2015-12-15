#include <boost/test/unit_test.hpp>
#include <smurf/Robot.hpp>
//#include <orocos_cpp/YAMLConfiguration.hpp>
#include <urdf_model/link.h>

#include <configmaps/ConfigData.h>
#include <smurf_parser/SMURFParser.h>

#include <boost/filesystem.hpp>

using namespace std;

//const string path = orocos_cpp::YAMLConfigParser::applyStringVariableInsertions("<%=ENV(AUTOPROJ_CURRENT_ROOT) %>/<%=ENV(ASGUARD4)%>");
//const string robotPath = orocos_cpp::YAMLConfigParser::applyStringVariableInsertions("<%=ENV(AUTOPROJ_CURRENT_ROOT) %>/<%=ENV(SPACECLIMBER)%>");
const string path = "";

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

BOOST_AUTO_TEST_CASE(test_load_collidables)
{
    configmaps::ConfigMap map;
    // parse joints from model
    boost::filesystem::path filepath(path);
    boost::shared_ptr<urdf::ModelInterface> model = smurf_parser::parseFile(&map, filepath.parent_path().generic_string(), filepath.filename().generic_string(), true);
    smurf::Robot robot;
    robot.loadFromSmurf(path); // This should load all the frames
    robot.loadCollisions();
    robot.loadCollidables();
}    