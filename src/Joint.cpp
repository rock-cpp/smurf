#include "Joint.hpp"

smurf::Joint::Joint(const std::string &name, smurf::Frame* sourceFrame, smurf::Frame* targetFrame, const std::string& provider, 
                    const std::string& port, const std::string& driverName, base::JointLimitRange& limits, 
                    const Eigen::Affine3d& sourceToAxis): 
                    DynamicTransformation(name, sourceFrame, targetFrame, provider, port), limits(limits), 
                    sourceToAxis(sourceToAxis)
{

}

smurf::Joint::Joint(const std::string &name, smurf::Frame* sourceFrame, smurf::Frame* targetFrame, const std::string& provider, 
                    const std::string& port, const std::string& driverName, base::JointLimitRange& limits, 
                    const Eigen::Affine3d& sourceToAxis, const Eigen::Affine3d& parentToJointOrigin): 
                    DynamicTransformation(name, sourceFrame, targetFrame, provider, port), limits(limits), 
                    sourceToAxis(sourceToAxis), parentToJointOrigin(parentToJointOrigin)
{

}

smurf::Joint::Joint(const std::string &name, smurf::Frame* sourceFrame, smurf::Frame* targetFrame, const std::string& provider, 
                    const std::string& port, const std::string& driverName, base::JointLimitRange& limits, 
                    const Eigen::Affine3d& sourceToAxis, const Eigen::Affine3d& parentToJointOrigin,
                    boost::shared_ptr<urdf::Joint> jointModel): 
                    DynamicTransformation(name, sourceFrame, targetFrame, provider, port), limits(limits), 
                    sourceToAxis(sourceToAxis), parentToJointOrigin(parentToJointOrigin),
                    jointModel(jointModel)
{

}


smurf::Joint::Joint(smurf::Frame* sourceFrame, smurf::Frame* targetFrame, const std::string& provider, 
                    const std::string& port, const std::string& driverName, base::JointLimitRange& limits, 
                    const Eigen::Affine3d& sourceToAxis): 
                    DynamicTransformation(sourceFrame, targetFrame, provider, port), limits(limits), 
                    sourceToAxis(sourceToAxis)
{

}

smurf::Joint::Joint(smurf::Frame* sourceFrame, smurf::Frame* targetFrame, const std::string& provider, 
                    const std::string& port, const std::string& driverName, base::JointLimitRange& limits, 
                    const Eigen::Affine3d& sourceToAxis, const Eigen::Affine3d& parentToJointOrigin): 
                    DynamicTransformation(sourceFrame, targetFrame, provider, port), limits(limits), 
                    sourceToAxis(sourceToAxis), parentToJointOrigin(parentToJointOrigin)
{

}

smurf::Joint::Joint(smurf::Frame* sourceFrame, smurf::Frame* targetFrame, const std::string& provider, 
                    const std::string& port, const std::string& driverName, base::JointLimitRange& limits, 
                    const Eigen::Affine3d& sourceToAxis, const Eigen::Affine3d& parentToJointOrigin,
                    boost::shared_ptr<urdf::Joint> jointModel): 
                    DynamicTransformation(sourceFrame, targetFrame, provider, port), limits(limits), 
                    sourceToAxis(sourceToAxis), parentToJointOrigin(parentToJointOrigin),
                    jointModel(jointModel)
{

}
const Eigen::Affine3d& smurf::Joint::getSourceToAxis() const
{
    return this -> sourceToAxis;
};

const Eigen::Affine3d& smurf::Joint::getParentToJointOrigin() const
{
    return this -> parentToJointOrigin;
};

boost::shared_ptr<urdf::Joint> smurf::Joint::getJointModel() const
{
    return this -> jointModel;
};
