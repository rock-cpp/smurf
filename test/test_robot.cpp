#include <boost/test/unit_test.hpp>
#include <smurf/Robot.hpp>
#include <urdf_model/link.h>

#include <configmaps/ConfigData.h>
#include <smurf_parser/SMURFParser.h>

#include <boost/filesystem.hpp>

using namespace std;

//const string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf";
//const string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes_dynamic_joint.smurf";
//const string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes_with_motor.smurf";
//const string path="./sample_smurfs/asguard_v4/smurf/asguard_v4.smurf";
// Test has to be run from test folder otherwise the smurf document won't be found
const string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes_with_sensor.smurf";

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
    boost::shared_ptr<urdf::ModelInterface> model =
        smurf_parser::parseFile(&map, filepath.parent_path().generic_string(),
                filepath.filename().generic_string(), true);
    smurf::Robot robot;
    robot.loadFromSmurf(path); // This one already loads the collidables and the inertials
}    

BOOST_AUTO_TEST_CASE(test_load_motors)
{
    configmaps::ConfigMap map;
    boost::filesystem::path filepath(path);
    boost::shared_ptr<urdf::ModelInterface> model =
        smurf_parser::parseFile(&map, filepath.parent_path().generic_string(),
                filepath.filename().generic_string(), true);
    smurf::Robot robot;
    robot.loadFromSmurf(path);
    std::vector<smurf::Motor *> motors = robot.getMotors();
    std::cout << motors.size() << std::endl;
    for(smurf::Motor* motor : motors)
    {
        std::cout << "Motor name is " << motor->getName() << std::endl;
        configmaps::ConfigMap map = motor -> getMotorMap();
        std::string mapString = map.toYamlString();
        std::cout << "String from map: " << mapString << std::endl;

    }
}

BOOST_AUTO_TEST_CASE(test_load_sensors)
{
    configmaps::ConfigMap map;
    boost::filesystem::path filepath(path);
    boost::shared_ptr<urdf::ModelInterface> model =
        smurf_parser::parseFile(&map, filepath.parent_path().generic_string(),
                filepath.filename().generic_string(), true);
    smurf::Robot robot;
    robot.loadFromSmurf(path);
    std::vector<smurf::Sensor *> sensors = robot.getSensors();
    std::cout << sensors.size() << std::endl;
    for(smurf::Sensor* sensor : sensors)
    {
        std::cout << "Sensor name is " << sensor->getName() << std::endl;
        configmaps::ConfigMap map = sensor -> getMap();
        std::string mapString = map.toYamlString();
        std::cout << "String from map: " << mapString << std::endl;
    }
}

BOOST_AUTO_TEST_CASE(test_get_rootFrame)
{
    const string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes_with_sensor.smurf";
    smurf::Robot robot;
    robot.loadFromSmurf(path);
    const smurf::Frame * rootFrame = robot.getRootFrame();
    const std::string rootFrameName = rootFrame->getName();
    BOOST_CHECK(rootFrameName == "root");
}

BOOST_AUTO_TEST_CASE(test_get_limits)
{
    const string path="./sample_smurfs/two_boxes_joined/smurf/two_boxes_with_motor.smurf";
    smurf::Robot robot;
    robot.loadFromSmurf(path);
    // only one joint
    const vector<smurf::Joint*> joints = robot.getJoints();
    BOOST_CHECK(joints.size() == 1);    

    std::pair<double, double> position_limit = joints.at(0)->getPositionLimits();
    std::pair<double, double> effort_limit = joints.at(0)->getEffortLimits();
    std::pair<double, double> speed_limit = joints.at(0)->getSpeedLimits();

    BOOST_CHECK(position_limit.first == -0.785398);
    BOOST_CHECK(position_limit.second == 0.785398);

    BOOST_CHECK(effort_limit.first == 0);
    BOOST_CHECK(effort_limit.second == 1000);

    BOOST_CHECK(speed_limit.first == 0);
    BOOST_CHECK(speed_limit.second == 6.28);        
}