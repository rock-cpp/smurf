#include "Robot.hpp"

#include <boost/filesystem.hpp>
#include <smurf_parser/SMURFParser.h>
#include <base/Logging.hpp>
#include <configmaps/ConfigData.h>

#include "Frame.hpp"
#include "RotationalJoint.hpp"
#include "TranslationalJoint.hpp"

std::string checkGet(configmaps::ConfigMap &map, const std::string &key)
{
    auto it = map.find(key);
    if(it == map.end())
    {
        return std::string();
        throw std::runtime_error("Smurf:: Error, could not find key " + key + " in config map");
    }
    
    return it->second;
}

smurf::Robot::Robot()
{

}

smurf::Frame* smurf::Robot::getFrameByName(const std::string& name)
{
    for(Frame *fr: availableFrames)
    {
        if(fr->getName() == name)
            return fr;
    }
    
    throw std::runtime_error("smurf::Robot::getFrameByName : Error , frame " + name + " is not known" );
}


void smurf::Robot::loadCollidables()
{
    for (configmaps::ConfigVector::iterator it = smurfMap["collision"].begin(); it != smurfMap["collision"].end(); ++it) 
    {
        configmaps::ConfigMap collidableMap = it->children;
        smurf::Collidable* collidable = new Collidable(collidableMap["name"], collidableMap["bitmask"] );
        Frame* frame = getFrameByName(collidableMap["link"]);
        frame->addCollidable(*collidable);
        LOG_DEBUG_S << "[smurf::Robot::loadCollidables] Loaded the collidable "<< static_cast<std::string>(collidableMap["name"]);
    }
}

/*
 *  Each link in the map has collision data
 * TODO: Make another method that loads the collidables
 * 
 */
void smurf::Robot::loadCollisions()
{
    LOG_DEBUG_S << " ********* :) ************ ";
    for(std::pair<std::string, boost::shared_ptr<urdf::Link>> link: model->links_)
    {
        LOG_DEBUG_S << "Found link =:-D  " << link.first;
        smurf::Frame* frame = getFrameByName(link.first);
        for(boost::shared_ptr<urdf::Collision> collision : link.second->collision_array)
        {
            frame->addCollision(*collision);
        }
        
    }
    LOG_DEBUG_S << " ********* :) ************ ";
    

}

void smurf::Robot::loadFromSmurf(const std::string& path)
{
    //configmaps::ConfigMap smurfMap;

    // parse joints from model
    boost::filesystem::path filepath(path);
    model = smurf_parser::parseFile(&smurfMap, filepath.parent_path().generic_string(), filepath.filename().generic_string(), true);
    
    //first we need to create all Frames
    for (configmaps::ConfigVector::iterator it = smurfMap["frames"].begin(); it != smurfMap["frames"].end(); ++it) 
    {
        configmaps::ConfigMap &fr(it->children);

        Frame *frame = new Frame(fr["name"]);
        availableFrames.push_back(frame);
        //std::cout << "Adding additional frame " << frame->getName() << std::endl;
    }
    
    for(std::pair<std::string, boost::shared_ptr<urdf::Link>> link: model->links_)
    {
        Frame *frame = new Frame(link.first);
        for(boost::shared_ptr<urdf::Visual> visual : link.second->visual_array)
        {
            frame->addVisual(*visual);
        }
        availableFrames.push_back(frame);
        

        //std::cout << "Adding link frame " << frame->getName() << std::endl;
    }

    for(std::pair<std::string, boost::shared_ptr<urdf::Joint> > jointIt: model->joints_)
    {
        boost::shared_ptr<urdf::Joint> joint = jointIt.second;
        //std::cout << "Adding joint " << joint->name << std::endl;
        
        Frame *source = getFrameByName(joint->parent_link_name);
        Frame *target = getFrameByName(joint->child_link_name);

        //TODO this might not be set in some cases, perhaps force a check
        configmaps::ConfigMap annotations;
        for(configmaps::ConfigItem &cv : smurfMap["joints"])
        {
            if(static_cast<std::string>(cv.children["name"]) == joint->name)
            {
                annotations = cv.children;
            }
        }
        switch(joint->type)
        {
            case urdf::Joint::FIXED:
            {
                const urdf::Pose &tr(joint->parent_to_joint_origin_transform);     
                StaticTransformation *transform = new StaticTransformation(source, target,
                                                                           Eigen::Quaterniond(tr.rotation.w, tr.rotation.x, tr.rotation.y, tr.rotation.z),
                                                                           Eigen::Vector3d(tr.position.x, tr.position.y, tr.position.z));              
                staticTransforms.push_back(transform);
            }
            break;
            case urdf::Joint::FLOATING:
            {
                DynamicTransformation *transform = new DynamicTransformation(source, target, checkGet(annotations, "provider"), checkGet(annotations, "port"));
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
                Joint *smurfJoint = new Joint (source, target, checkGet(annotations, "provider"), checkGet(annotations, "port"), checkGet(annotations, "driver"), limits, sourceToAxis, parentToOriginAff, joint); 
                joints.push_back(smurfJoint);
            }
            break;
            case urdf::Joint::REVOLUTE:
            case urdf::Joint::CONTINUOUS:
            case urdf::Joint::PRISMATIC:
            {
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
                    transform = new RotationalJoint(source, target, checkGet(annotations, "provider"), checkGet(annotations, "port"), checkGet(annotations, "driver"), limits, sourceToAxis, axis, parentToOriginAff, joint);
                    smurfJoint = (Joint *)transform;
                }
                else
                {
                    transform = new TranslationalJoint(source, target, checkGet(annotations, "provider"), checkGet(annotations, "port"), checkGet(annotations, "driver"), limits, sourceToAxis, axis, parentToOriginAff, joint);
                    smurfJoint = (Joint *)transform;
                }
                dynamicTransforms.push_back(transform);
                joints.push_back(smurfJoint);
            }
            break;
            default:
                throw std::runtime_error("Smurf: Error, got unhandles Joint type");
        }
    }

    
    // parse sensors from map
    for (configmaps::ConfigVector::iterator it = smurfMap["sensors"].begin(); it != smurfMap["sensors"].end(); ++it) 
    {
        configmaps::ConfigMap sensorMap = it->children;
        smurf::Sensor *sensor = new Sensor(sensorMap["name"], sensorMap["type"], sensorMap["taskInstanceName"], getFrameByName(sensorMap["link"]));
        sensors.push_back(sensor);
    }
}




