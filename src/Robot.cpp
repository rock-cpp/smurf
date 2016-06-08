#include "Robot.hpp"

#include <boost/filesystem.hpp>
#include <smurf_parser/SMURFParser.h>
#include <base/Logging.hpp>

#include "Frame.hpp"
#include "RotationalJoint.hpp"
#include "TranslationalJoint.hpp"

smurf::Robot::Robot()
{
    smurfMap = new configmaps::ConfigMap();
}

smurf::Robot::~Robot()
{

}

std::string checkGet(configmaps::ConfigMap &map, const std::string &key)
{
    auto it = map.find(key);
    if(it == map.end())
    {
        throw std::runtime_error("Smurf:: Error, could not find key " + key + " in config map");
    }
    
    return it->second;
}


// Private
smurf::Frame* smurf::Robot::getFrameByName(const std::string& name)
{
    for(Frame *fr: availableFrames)
    {
        if(fr->getName() == name)
            return fr;
    }
    
    throw std::runtime_error("smurf::Robot::getFrameByName : Error , frame " + name + " is not known" );
}

const mars::interfaces::contact_params smurf::Robot::getContactParams(const std::string& collisionName, const std::string& linkName)
{
    mars::interfaces::contact_params result;
    result.setZero();
    bool found = false;
    configmaps::ConfigVector::iterator it = (*smurfMap)["collision"].begin();
    while ((! found) and (it != (*smurfMap)["collision"].end()))
    {
        configmaps::ConfigMap &collidableMap(*it);
        std::string name = static_cast<std::string>(collidableMap["name"]);
        std::string link = static_cast<std::string>(collidableMap["link"]);
        if ((name == collisionName) and (linkName == link))
        {
            found = true;
            double stored_cfm = static_cast<double>(collidableMap["ccfm"]);
            // When not set in the smurf, the returned value for the cfm is 0.0
            if (stored_cfm != 0.0) 
            { 
              result.cfm = static_cast<double>(collidableMap["ccfm"]); 
            }
            result.coll_bitmask = static_cast<int>(collidableMap["bitmask"]);
            if(debug)
            {
                LOG_DEBUG_S << "[smurf::Robot::getContactParams] Found the ccfm ("<< result.cfm <<")correspondent to the collisionName "<< collisionName;
                LOG_DEBUG_S << "[smurf::Robot::getContactParams] Found the bitmask ("<< result.coll_bitmask <<")correspondent to the collisionName "<< collisionName;
            }
        }
        ++it;
    }
    return result;
}
    
void smurf::Robot::loadCollidables()
{

    if (debug) {LOG_DEBUG_S << "[smurf::Robot::loadCollidables] Loading collidables just started ";}
    for(std::pair<std::string, boost::shared_ptr<urdf::Link>> link: model->links_)
    {
        smurf::Frame* frame = getFrameByName(link.first);
        for(boost::shared_ptr<urdf::Collision> collision : link.second->collision_array)
        {
            // Find the correspondent collidable data if exists and create the collidable object
            smurf::Collidable* collidable = new Collidable(collision->name, getContactParams(collision->name, link.first), *collision );
            frame->addCollidable(*collidable);
        }
    }
}

void smurf::Robot::loadCollisions()
{
    for(std::pair<std::string, boost::shared_ptr<urdf::Link>> link: model->links_)
    {
        smurf::Frame* frame = getFrameByName(link.first);
        for(boost::shared_ptr<urdf::Collision> collision : link.second->collision_array)
        {
            frame->addCollision(*collision);
        }
    }
}

void smurf::Robot::loadInertials()
{
    for(std::pair<std::string, boost::shared_ptr<urdf::Link>> link: model->links_)
    {
        smurf::Frame* frame = getFrameByName(link.first);
        if (debug) { LOG_DEBUG_S << " [smurf::Robot::loadInertials] Checking for inertials in link with name " << link.first;}
        if (link.second->inertial != NULL)
        {
            smurf::Inertial inertial(*link.second->inertial);
            frame->setInertial(inertial);
            if (debug) { LOG_DEBUG_S << " [smurf::Robot::loadInertials] " << link.first << ": Inertial found in the link";}
        }
    }
}

configmaps::ConfigMap smurf::Robot::getAnnotations(const boost::shared_ptr<urdf::Joint>& joint)
{
    bool foundAnnotation = false;
    configmaps::ConfigMap annotations;
    for(configmaps::ConfigItem &cv : (*smurfMap)["joint_tasks"])
    {
        if(static_cast<std::string>((cv)["name"]) == joint->name)
        {
            annotations = cv;
            foundAnnotation = true;
            break;
        }
    }    
    if(!foundAnnotation)
    {
        throw std::runtime_error("Could not find annotation for joint " + joint->name);
    }
    return annotations;
}

void smurf::Robot::loadJoints()
{
    for(std::pair<std::string, boost::shared_ptr<urdf::Joint> > jointIt: model->joints_)
    {
        boost::shared_ptr<urdf::Joint> joint = jointIt.second;
        Frame *source = getFrameByName(joint->parent_link_name);
        Frame *target = getFrameByName(joint->child_link_name);
        switch(joint->type)
        {
          case urdf::Joint::FIXED:
            {
                const urdf::Pose &tr(joint->parent_to_joint_origin_transform);     
                StaticTransformation *transform = new StaticTransformation(joint->name, source, target,
                                                                           Eigen::Quaterniond(tr.rotation.w, tr.rotation.x, tr.rotation.y, tr.rotation.z),
                                                                           Eigen::Vector3d(tr.position.x, tr.position.y, tr.position.z));              
                if (debug) {LOG_DEBUG_S << "[smurf::Robot::loadJoint] Pushing back the statict transformation for the fixed joint" << joint->name;}
                staticTransforms.push_back(transform);
            }
            break;
            case urdf::Joint::FLOATING:
            {
                configmaps::ConfigMap annotations = getAnnotations(joint);
                DynamicTransformation *transform = new DynamicTransformation(joint->name, source, target, checkGet(annotations, "provider"), checkGet(annotations, "port"));
                dynamicTransforms.push_back(transform);
                Eigen::Vector3d axis(joint->axis.x, joint->axis.y, joint->axis.z);
                Eigen::Affine3d sourceToAxis(Eigen::Affine3d::Identity());
                sourceToAxis.translation() = axis;
                base::JointLimitRange limits;
                const urdf::Pose parentToOrigin(joint->parent_to_joint_origin_transform);
                Eigen::Quaterniond rot(parentToOrigin.rotation.w, parentToOrigin.rotation.x, parentToOrigin.rotation.y, parentToOrigin.rotation.z);
                Eigen::Vector3d trans(parentToOrigin.position.x, parentToOrigin.position.y, parentToOrigin.position.z);
                Eigen::Affine3d parentToOriginAff;
                parentToOriginAff.setIdentity();
                parentToOriginAff.rotate(rot);
                parentToOriginAff.translation() = trans;
                Joint *smurfJoint = new Joint (joint->name, source, target, checkGet(annotations, "provider"), checkGet(annotations, "port"), checkGet(annotations, "driver"), limits, sourceToAxis, parentToOriginAff, joint); 
                joints.push_back(smurfJoint);
            }
            break;
            case urdf::Joint::REVOLUTE:
            case urdf::Joint::CONTINUOUS:
            case urdf::Joint::PRISMATIC:
            {
                configmaps::ConfigMap annotations = getAnnotations(joint);
                base::JointState minState;
                minState.position = joint->limits->lower;
                minState.effort = 0;
                minState.speed = 0;
                
                base::JointState maxState;
                maxState.position = joint->limits->upper;
                maxState.effort = joint->limits->effort;
                maxState.speed = joint->limits->velocity;
                
                base::JointLimitRange limits;
                limits.min = minState;
                limits.max = maxState;
                
                Eigen::Vector3d axis(joint->axis.x, joint->axis.y, joint->axis.z);
                Eigen::Affine3d sourceToAxis(Eigen::Affine3d::Identity());
                sourceToAxis.translation() = axis;
                
                DynamicTransformation *transform = NULL;
                Joint *smurfJoint;
                // push the correspondent smurf::joint 
                const urdf::Pose parentToOrigin(joint->parent_to_joint_origin_transform);
                Eigen::Quaterniond rot(parentToOrigin.rotation.w, parentToOrigin.rotation.x, parentToOrigin.rotation.y, parentToOrigin.rotation.z);
                Eigen::Vector3d trans(parentToOrigin.position.x, parentToOrigin.position.y, parentToOrigin.position.z);
                Eigen::Affine3d parentToOriginAff;
                parentToOriginAff.setIdentity();
                parentToOriginAff.rotate(rot);
                parentToOriginAff.translation() = trans;
                if(joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::CONTINUOUS)
                {
                    transform = new RotationalJoint(joint->name, source, target, checkGet(annotations, "provider"), checkGet(annotations, "port"), checkGet(annotations, "driver"), limits, sourceToAxis, axis, parentToOriginAff, joint);
                }
                else
                {
                    transform = new TranslationalJoint(joint->name, source, target, checkGet(annotations, "provider"), checkGet(annotations, "port"), checkGet(annotations, "driver"), limits, sourceToAxis, axis, parentToOriginAff, joint);
                }
                smurfJoint = (Joint *)transform;
                if (debug) {LOG_DEBUG_S << "[smurf::Robot::loadJoint] Pushing back the dynamic transformation for revolute or continuous joint" << joint->name;}
                dynamicTransforms.push_back(transform);
                joints.push_back(smurfJoint);
            }
            break;
            default:
                throw std::runtime_error("Smurf: Error, got unhandles Joint type");
        }
    }

}

void smurf::Robot::loadMotors()
{
    for (configmaps::ConfigVector::iterator it = (*smurfMap)["motors"].begin(); it != (*smurfMap)["motors"].end(); ++it) 
    {
        configmaps::ConfigMap motorMap = *it;
        smurf::Motor *motor = new Motor(motorMap);
        motors.push_back(motor);
        if (debug) { LOG_DEBUG_S << " [smurf::Robot::loadMotor] A motor found with name: " << motor->getName() ;}
    }
}

void smurf::Robot::loadSensors()
{
    // parse sensors from map
    for (configmaps::ConfigVector::iterator it = (*smurfMap)["sensors"].begin(); it != (*smurfMap)["sensors"].end(); ++it) 
    {
        configmaps::ConfigMap sensorMap = *it;
        smurf::Sensor *sensor = new Sensor(sensorMap["name"], sensorMap["type"], sensorMap["taskInstanceName"], getFrameByName(sensorMap["link"]), sensorMap);
        sensors.push_back(sensor);
    }
    
}

void smurf::Robot::loadFrames(boost::shared_ptr< urdf::ModelInterface > model)
{
    boost::shared_ptr<const urdf::Link> root = model->getRoot();
    const std::string rootName = root->name;
    for(std::pair<std::string, boost::shared_ptr<urdf::Link>> link: model->links_)
    {
        Frame *frame = new Frame(link.first);
        availableFrames.push_back(frame);
        if (frame->getName() == rootName)
        {
            if (debug){LOG_DEBUG_S << "[smurf::Robot::LoadFrames] Found the root frame: " << root->name;}
            rootFrame = frame;
        }
    }
}

void smurf::Robot::loadVisuals()
{
    for(std::pair<std::string, boost::shared_ptr<urdf::Link>> link: model->links_)
    {
        Frame *frame = getFrameByName(link.second->name);
        for(boost::shared_ptr<urdf::Visual> visual : link.second->visual_array)
        {
            frame->addVisual(*visual);
        }
    }
}

void smurf::Robot::loadFromSmurf(const std::string& path)
{    
    // Load model from file
    boost::filesystem::path filepath(path);
    model = smurf_parser::parseFile(smurfMap, filepath.parent_path().generic_string(), filepath.filename().generic_string(), true);
    loadFrames(model); //NOTE Sets also the root frame
    loadVisuals();
    loadJoints(); 
    loadSensors();
    loadCollidables();
    loadInertials();
    loadMotors();
}
