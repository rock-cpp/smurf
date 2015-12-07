#ifndef SMURF_H
#define SMURF_H

#include <base/Eigen.hpp>
#include <base/JointLimitRange.hpp>
#include <base/JointState.hpp>
#include <base/JointTransform.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <urdf_model/model.h>

namespace smurf
{

class Collidable
{
};


class Frame
{
public:
    Frame(const std::string &name, const std::vector<urdf::Visual>& visuals);
    Frame(const std::string &name);
    Frame();

    const std::string getName() const
    {
        return name;
    };

    void getCollisionObjects(std::vector<smurf::Collidable> &CollisionObjects);
    std::vector<Collidable> &getCollisionObjects();
    
    void addVisual(const urdf::Visual& visual);
    void setVisuals(const std::vector<urdf::Visual>& visuals);
    void getVisuals(std::vector<urdf::Visual>& Visuals) const;
    std::vector<urdf::Visual>& getVisuals();
    
private:
    ///Name of the frame
    std::string name;
    
    ///Collision objects inside the frame
    std::vector<Collidable> collisionObjects;
    
    ///Visuals that can be displayed inside the frame
    std::vector<urdf::Visual> visuals;
    
    ///TODO add additional data in map or whatever
    
};

class Sensor
{
public:
    Sensor(const std::string &name, const std::string &type, const std::string &taskInstanceName, Frame *inFrame);
    Sensor();
    std::string getname();
    std::string gettype();
    std::string gettaskInstanceName();
    Frame * getattachmentPoint();
private:
    std::string name;
    std::string type;

    std::string taskInstanceName;

    Frame *attachmentPoint;
};

class Transformation
{
public:
    
    Transformation(Frame *sourceFrame, Frame *targetFrame);

    const Frame &getSourceFrame() const
    {
        return *sourceFrame;
    }
    
    const Frame &getTargetFrame() const
    {
        return *targetFrame;
    }

    const std::string &getName() const
    {
        return name;
    }
    
private:
    
    /**
     * Name of the Transformation, defaults to
     * "<sourceFrameName>2<TargetFrameName>"
     * */
    std::string name;
    
    Frame *sourceFrame;
    Frame *targetFrame;
};

class StaticTransformation : public Transformation
{
public:
    StaticTransformation(Frame* sourceFrame, Frame* targetFrame, const Eigen::Affine3d &sourceToTarget);
    StaticTransformation(Frame* sourceFrame, Frame* targetFrame, const Eigen::Quaterniond &rotation, const Eigen::Vector3d &translation);
    
    const Eigen::Affine3d &getTransformation() const;

private:
    /**
     * Transformation from the source frame
     * to the target frame.
     * */
    Eigen::Affine3d sourceToTarget;
};

class DynamicTransformation : public Transformation
{
public:
    DynamicTransformation(Frame *sourceFrame, Frame *targetFrame, const std::string &provider, const std::string &port);
    
    const std::string &getProviderName() const
    {
        return providerName;
    };
    
    const std::string &getProviderPortName() const
    {
        return providerPortName;
    };
    
private:
    /**
     * Name of the task instance that provides the
     * dynamic transformation
     * */
    std::string providerName;
    
    /**
     * Name of the port on the task instance, that
     * provides the transformation.
     * */
    std::string providerPortName;
};

class Joint : public DynamicTransformation
{
public:
    
    Joint(Frame* sourceFrame, Frame* targetFrame, const std::string &provider, const std::string &port, const std::string &driverName, base::JointLimitRange &limits, const Eigen::Affine3d &sourceToAxis);
    
    Joint(Frame* sourceFrame, Frame* targetFrame, const std::string& provider, const std::string& port, const std::string& driverName, base::JointLimitRange& limits, const Eigen::Affine3d& sourceToAxis, const Eigen::Affine3d& parentToJointOrigin); 
    
    Joint(Frame* sourceFrame, Frame* targetFrame, const std::string& provider, const std::string& port, const std::string& driverName, base::JointLimitRange& limits, const Eigen::Affine3d& sourceToAxis, const Eigen::Affine3d& parentToJointOrigin, boost::shared_ptr<urdf::Joint> jointModel); 
    
    const Eigen::Affine3d &getSourceToAxis() const;
    
    const Eigen::Affine3d &getParentToJointOrigin() const;
    
    void setParentToJointOrigin(const Eigen::Affine3d inParentToJointOrigin);
    
    boost::shared_ptr<urdf::Joint> getJointModel() const;
    

    
protected:

    /**
     * Name of the rock task that provides 
     * the driver for this joint.
     * 
     * e.g. "servo_dynamixel::Task"
     * */
    std::string driverName;
    
    /**
     * Physical limits of the joint.
     * */
    base::JointLimitRange limits;
    
    /**
     * Transformation from the source frame
     * to the joint axis.
     * */
    Eigen::Affine3d sourceToAxis;
    
    Eigen::Affine3d parentToJointOrigin;
    
    /**
     * Urdf data of the joint
     * 
     */
    boost::shared_ptr<urdf::Joint> jointModel;

};

class RotationalJoint : public Joint
{
public:
    RotationalJoint(Frame* sourceFrame, Frame* targetFrame, const std::string& provider, const std::string& port, const std::string& driverName, base::JointLimitRange& limits, const Eigen::Affine3d& sourceToAxis,  const Eigen::Vector3d &rotationAxis);
    
    RotationalJoint(Frame* sourceFrame, Frame* targetFrame, const std::string& provider, const std::string& port, const std::string& driverName, base::JointLimitRange& limits, const Eigen::Affine3d& sourceToAxis, const Eigen::Vector3d& rotationAxis, const Eigen::Affine3d& parentToJointOrigin, boost::shared_ptr<urdf::Joint> jointModel); 

    /**
     * Rotation axis of the joint in the target frame.
     * */
    Eigen::Vector3d rotationAxis;
};

class TranslationalJoint : public Joint
{
public:
    TranslationalJoint(Frame* sourceFrame, Frame* targetFrame, const std::string& provider, const std::string& port, const std::string& driverName, base::JointLimitRange& limits, const Eigen::Affine3d& sourceToAxis, const Eigen::Vector3d &translationAxis);
    
    TranslationalJoint(smurf::Frame* sourceFrame, smurf::Frame* targetFrame, const std::string& provider, const std::string& port, const std::string& driverName, base::JointLimitRange& limits, const Eigen::Affine3d& sourceToAxis, const Eigen::Vector3d& translationAxis, const Eigen::Affine3d& parentToJointOrigin, boost::shared_ptr<urdf::Joint> jointModel);
    
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
    
    const std::vector<StaticTransformation *> & getStaticTransforms() const
    {
        return staticTransforms;
    };
    
    const std::vector<DynamicTransformation *> & getDynamicTransforms() const
    {
        return dynamicTransforms;
    };

    const std::vector<Joint *> & getJoints() const
    {
        return joints;
    };

    const std::vector<Sensor *> & getSensors() const
    {
        return sensors;
    };

    const std::vector<Frame *> & getFrames() const
    {
        return availableFrames;
    };
    
    const Frame* getRootFrame() const
    {
        return rootFrame;
    };
    
protected:
    
    Frame *getFrameByName(const std::string &name);
    
    Frame *rootFrame;
    boost::shared_ptr<urdf::ModelInterface> model;

    std::vector<Frame *> availableFrames;
    std::vector<StaticTransformation *> staticTransforms;
    std::vector<DynamicTransformation *> dynamicTransforms;
    std::vector<Joint *> joints;
    std::vector<Sensor *> sensors;
};

};
#endif // SMURF_H
