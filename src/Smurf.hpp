#ifndef SMURF_H
#define SMURF_H

#include <base/Eigen.hpp>
#include <base/JointLimitRange.hpp>
#include <base/JointState.hpp>
#include <base/JointTransform.hpp>
#include <base/samples/RigidBodyState.hpp>


namespace smurf
{

class Collidable
{
};

class Visual
{
};

class Frame
{
    ///Name of the frame
    std::string name;
    
    ///Collision objects inside the frame
    std::vector<Collidable> collisionObjects;
    
    ///Visuals that can be displayed inside the frame
    std::vector<Visual> visuals;
    
    ///TODO add additional data in map or whatever
    
};

class Sensor
{
    std::string name;
    std::string type;

    Frame *attachmentPoint;
};

class Transformation
{
    /**
     * Name of the Transformation, defaults to
     * "<sourceFrameName>2<TargetFrameName>"
     * */
    
    std::string name;
    
    Frame *sourceFrame;
    Frame *targetFrame;
    
    /**
     * Transformation from the source frame
     * to the target frame.
     * */
    Eigen::Affine3d sourceToTarget;
};

class Joint : public Transformation
{
    /**
     * Name of the rock task that provides 
     * the driver for this joint.
     * */
    std::string driverName;
    
    /**
     * Physical limits of the joint.
     * */
    base::JointLimitRange limits;
};

class RotationalJoint : public Joint
{
    /**
     * Rotation axis of the joint in the target frame.
     * */
    Eigen::Vector3d rotationAxis;
};

class TranslationalJoint : public Joint
{
    /**
     * Sliding axis of the joint in the target frame.
     * */
    Eigen::Vector3d translationAxis;
};

class Robot
{
public:
    Robot();
    
    void loadFromSmurf(const std::string &path);
    
    Frame rootFrame;
    
    std::vector<Frame> availableFrames;
    
    std::vector<Transformation *> staticTransforms;
    std::vector<Joint *> joints;
    std::vector<Sensor> sensors;
};

};
#endif // SMURF_H
