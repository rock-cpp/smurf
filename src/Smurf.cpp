#include "Smurf.hpp"
#include <smurf_parser/SMURFParser.h>
#include <boost/filesystem.hpp>
#include <configmaps/ConfigData.h>

smurf::Frame::Frame(const std::string& name): name(name)
{

}

smurf::Frame::Frame()
{

}


smurf::Joint::Joint(smurf::Frame* sourceFrame, smurf::Frame* targetFrame, const std::string& provider, 
                    const std::string& port, const std::string& driverName, base::JointLimitRange& limits, 
                    const Eigen::Affine3d& sourceToAxis): 
                    DynamicTransformation(sourceFrame, targetFrame, provider, port), limits(limits), 
                    sourceToAxis(sourceToAxis)
{

}

const Eigen::Affine3d& smurf::Joint::getAxisTransformation() const
{
    return sourceToAxis;
};

smurf::RotationalJoint::RotationalJoint(smurf::Frame* sourceFrame, smurf::Frame* targetFrame, 
                                        const std::string& provider, const std::string& port, 
                                        const std::string& driverName, base::JointLimitRange& limits, 
                                        const Eigen::Affine3d& sourceToAxis, const Eigen::Vector3d& rotationAxis): 
                                        Joint(sourceFrame, targetFrame, provider, port, driverName, limits, sourceToAxis), 
                                        rotationAxis(rotationAxis)
{

}

smurf::TranslationalJoint::TranslationalJoint(smurf::Frame* sourceFrame, smurf::Frame* targetFrame, const std::string& provider, 
                                              const std::string& port, const std::string& driverName, base::JointLimitRange& limits, 
                                              const Eigen::Affine3d& sourceToAxis, const Eigen::Vector3d& translationAxis): 
                                              Joint(sourceFrame, targetFrame, provider, port, driverName, limits, sourceToAxis), 
                                              translationAxis(translationAxis)
{

}

smurf::Sensor::Sensor()
{


}

smurf::Sensor::Sensor(const std::string &name, const std::string &type, const std::string &taskInstanceName, Frame *inFrame)
{
    this->name=name;
    this->type=type;
    this->taskInstanceName=taskInstanceName;
    this->attachmentPoint=inFrame;
}

std::string smurf::Sensor::getname()
{
    return this->name;
}

std::string smurf::Sensor::gettype()
{
    return this->type;
}

smurf::Frame * smurf::Sensor::getattachmentPoint()
{
    return this->attachmentPoint;
}

std::string smurf::Sensor::gettaskInstanceName()
{
    return this->taskInstanceName;
}


smurf::StaticTransformation::StaticTransformation(smurf::Frame* sourceFrame, smurf::Frame* targetFrame, const Eigen::Affine3d& sourceToTarget): 
    Transformation(sourceFrame, targetFrame), sourceToTarget(sourceToTarget)
{

}
//smurf::StaticTransformation::StaticTransformation()
//{

//}



smurf::StaticTransformation::StaticTransformation(smurf::Frame* sourceFrame, smurf::Frame* targetFrame, const Eigen::Quaterniond& rotation, const Eigen::Vector3d& translation): 
    Transformation(sourceFrame, targetFrame)
{
    sourceToTarget.setIdentity();
    sourceToTarget.rotate(rotation);
    sourceToTarget.translation() = translation;
}

const Eigen::Affine3d& smurf::StaticTransformation::getTransformation() const
{
    return sourceToTarget;
}

smurf::DynamicTransformation::DynamicTransformation(smurf::Frame* sourceFrame, smurf::Frame* targetFrame, const std::string& provider, const std::string& port): 
    Transformation(sourceFrame, targetFrame), providerName(provider), providerPortName(port)
{

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

void smurf::Robot::loadFromSmurf(const std::string& path)
{
    configmaps::ConfigMap map;

    // parse joints from model
    boost::filesystem::path filepath(path);
    model = smurf_parser::parseFile(&map, filepath.parent_path().generic_string(), filepath.filename().generic_string(), true);
    
    //first we need to create all Frames
    for (configmaps::ConfigVector::iterator it = map["frames"].begin(); it != map["frames"].end(); ++it) 
    {
        configmaps::ConfigMap &fr(it->children);

        Frame *frame = new Frame(fr["name"]);
        availableFrames.push_back(frame);
        std::cout << "Adding additional frame " << frame->getName() << std::endl;
    }
    
    for(std::pair<std::string, boost::shared_ptr<urdf::Link>> link: model->links_)
    {
        Frame *frame = new Frame(link.first);
        availableFrames.push_back(frame);
        std::cout << "Adding link frame " << frame->getName() << std::endl;
    }

    for(std::pair<std::string, boost::shared_ptr<urdf::Joint> > jointIt: model->joints_)
    {
        boost::shared_ptr<urdf::Joint> joint = jointIt.second;
        std::cout << "Adding joint " << joint->name << std::endl;
        
        Frame *source = getFrameByName(joint->parent_link_name);
        Frame *target = getFrameByName(joint->child_link_name);

        //TODO this might not be set in some cases, perhaps force a check
        configmaps::ConfigMap annotations;
        for(configmaps::ConfigItem &cv : map["joints"])
        {
            if(static_cast<std::string>(cv.children["name"]) == joint->name)
            {
                annotations = cv.children;
            }
        }
        
        //Add all transformations as static (as initial state)
        const urdf::Pose &tr(joint->parent_to_joint_origin_transform);
        
        StaticTransformation *transform = new StaticTransformation(source, target,
                                                    Eigen::Quaterniond(tr.rotation.w, tr.rotation.x, tr.rotation.y, tr.rotation.z),
                                                    Eigen::Vector3d(tr.position.x, tr.position.y, tr.position.z));
        
        std::cout << "Static Transformation " << transform->getName() << std::endl;
        staticTransforms.push_back(transform);

        switch(joint->type)
        {
            case urdf::Joint::FIXED:
            {
            }
            break;
            case urdf::Joint::FLOATING:
            {
                DynamicTransformation *transform = new DynamicTransformation(source, target, checkGet(annotations, "provider"), checkGet(annotations, "port"));
                dynamicTransforms.push_back(transform);
                Eigen::Vector3d axis(joint->axis.x, joint->axis.y, joint->axis.z);
                Eigen::Affine3d sourceToAxis(Eigen::Affine3d::Identity());
                sourceToAxis.translation() = axis;
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
                DynamicTransformation *transform = NULL;
                if(joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::CONTINUOUS)
                {
                    transform = new RotationalJoint(source, target, checkGet(annotations, "provider"), checkGet(annotations, "port"), checkGet(annotations, "driver"), limits, sourceToAxis, axis);
                }
                else
                {
                    transform = new TranslationalJoint(source, target, checkGet(annotations, "provider"), checkGet(annotations, "port"), checkGet(annotations, "driver"), limits, sourceToAxis, axis);
                }
                dynamicTransforms.push_back(transform);
            }
            break;
            default:
                throw std::runtime_error("Smurf: Error, got unhandles Joint type");
        }
    }

    
    // parse sensors from map
    for (configmaps::ConfigVector::iterator it = map["sensors"].begin(); it != map["sensors"].end(); ++it) 
    {
        configmaps::ConfigMap sensorMap = it->children;
        smurf::Sensor *sensor = new Sensor(sensorMap["name"], sensorMap["type"], sensorMap["taskInstanceName"], getFrameByName(sensorMap["link"]));
        sensors.push_back(sensor);
    }
}

smurf::Transformation::Transformation(smurf::Frame* sourceFrame, smurf::Frame* targetFrame) : name(sourceFrame->getName() + "2" + targetFrame->getName()), sourceFrame(sourceFrame), targetFrame(targetFrame)
{

}


void smurf::Frame::getCollisionObjects(std::vector<smurf::Collidable> &CollisionObjects)
{
    CollisionObjects=this->collisionObjects;
}
std::vector<smurf::Collidable> &smurf::Frame::getCollisionObjects()
{
    return this->collisionObjects;
}

void smurf::Frame::getVisuals(std::vector<smurf::Visual> & Visuals)
{
     Visuals=this->visuals;
}

std::vector<smurf::Visual> &smurf::Frame::getVisuals()
{
    return this->visuals;
}
