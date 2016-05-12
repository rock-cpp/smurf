#ifndef TRANSATIONALJOINT_H
#define TRANSATIONALJOINT_H

#include "Joint.hpp"

namespace smurf{
    
    class TranslationalJoint : public Joint
    {
    public:
        TranslationalJoint(const std::string &name, smurf::Frame* sourceFrame, smurf::Frame* targetFrame, const std::string& provider, const std::string& port, const std::string& driverName, base::JointLimitRange& limits, const Eigen::Affine3d& sourceToAxis, const Eigen::Vector3d &translationAxis);
        
        TranslationalJoint(const std::string &name, smurf::Frame* sourceFrame, smurf::Frame* targetFrame, const std::string& provider, const std::string& port, const std::string& driverName, base::JointLimitRange& limits, const Eigen::Affine3d& sourceToAxis, const Eigen::Vector3d& translationAxis, const Eigen::Affine3d& parentToJointOrigin, boost::shared_ptr<urdf::Joint> jointModel);
        
        /**
         * Sliding axis of the joint in the target frame.
         * */
        Eigen::Vector3d translationAxis;
    };
    
};

#endif // TRANSATIONALJOINT_H
