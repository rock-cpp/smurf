#ifndef ROTATIONALJOINT_H
#define ROTATIONALJOINT_H

#include "Joint.hpp"

namespace smurf
{
    
    class RotationalJoint : public Joint
    {
    public:
        RotationalJoint(const std::string name, Frame* sourceFrame, Frame* targetFrame, const std::string& provider, const std::string& port, const std::string& driverName, base::JointLimitRange& limits, const Eigen::Affine3d& sourceToAxis,  const Eigen::Vector3d &rotationAxis);
        
        RotationalJoint(const std::string name, Frame* sourceFrame, Frame* targetFrame, const std::string& provider, const std::string& port, const std::string& driverName, base::JointLimitRange& limits, const Eigen::Affine3d& sourceToAxis, const Eigen::Vector3d& rotationAxis, const Eigen::Affine3d& parentToJointOrigin, urdf::JointSharedPtr jointModel);
        
        RotationalJoint(const std::string name, Frame* sourceFrame, Frame* targetFrame, base::JointLimitRange& limits, const Eigen::Affine3d& sourceToAxis, const Eigen::Vector3d& rotationAxis, const Eigen::Affine3d& parentToJointOrigin, urdf::JointSharedPtr jointModel);

        RotationalJoint(Frame* sourceFrame, Frame* targetFrame, const std::string& provider, const std::string& port, const std::string& driverName, base::JointLimitRange& limits, const Eigen::Affine3d& sourceToAxis,  const Eigen::Vector3d &rotationAxis);
        
        RotationalJoint(Frame* sourceFrame, Frame* targetFrame, const std::string& provider, const std::string& port, const std::string& driverName, base::JointLimitRange& limits, const Eigen::Affine3d& sourceToAxis, const Eigen::Vector3d& rotationAxis, const Eigen::Affine3d& parentToJointOrigin, urdf::JointSharedPtr jointModel);
        /**
         * Rotation axis of the joint in the target frame.
         * */
        Eigen::Vector3d rotationAxis;
    };
    
};

#endif // ROTATIONALJOINT_H
