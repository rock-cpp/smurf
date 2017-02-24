#include "Joint.hpp"

smurf::Joint::Joint(const std::string &name, smurf::Frame* sourceFrame, smurf::Frame* targetFrame, const std::string& provider, 
                    const std::string& port, const std::string& driverName, base::JointLimitRange& limits, 
                    const Eigen::Affine3d& sourceToAxis): 
                    DynamicTransformation(name, sourceFrame, targetFrame, provider, port), limits(limits), 
                    sourceToAxis(sourceToAxis), isDynamic(false)
{

}

smurf::Joint::Joint(const std::string &name, smurf::Frame* sourceFrame, smurf::Frame* targetFrame, const std::string& provider, 
                    const std::string& port, const std::string& driverName, base::JointLimitRange& limits, 
                    const Eigen::Affine3d& sourceToAxis, const Eigen::Affine3d& parentToJointOrigin): 
                    DynamicTransformation(name, sourceFrame, targetFrame, provider, port), limits(limits), 
                    sourceToAxis(sourceToAxis), parentToJointOrigin(parentToJointOrigin), isDynamic(false)
{

}

smurf::Joint::Joint(const std::string &name, smurf::Frame* sourceFrame, smurf::Frame* targetFrame, const std::string& provider, 
                    const std::string& port, const std::string& driverName, base::JointLimitRange& limits, 
                    const Eigen::Affine3d& sourceToAxis, const Eigen::Affine3d& parentToJointOrigin,
                    urdf::JointSharedPtr jointModel):
                    DynamicTransformation(name, sourceFrame, targetFrame, provider, port), limits(limits), 
                    sourceToAxis(sourceToAxis), parentToJointOrigin(parentToJointOrigin),
                    jointModel(jointModel), isDynamic(false)
{

}


smurf::Joint::Joint(smurf::Frame* sourceFrame, smurf::Frame* targetFrame, const std::string& provider, 
                    const std::string& port, const std::string& driverName, base::JointLimitRange& limits, 
                    const Eigen::Affine3d& sourceToAxis): 
                    DynamicTransformation(sourceFrame, targetFrame, provider, port), limits(limits), 
                    sourceToAxis(sourceToAxis), isDynamic(false)
{

}

smurf::Joint::Joint(smurf::Frame* sourceFrame, smurf::Frame* targetFrame, const std::string& provider, 
                    const std::string& port, const std::string& driverName, base::JointLimitRange& limits, 
                    const Eigen::Affine3d& sourceToAxis, const Eigen::Affine3d& parentToJointOrigin): 
                    DynamicTransformation(sourceFrame, targetFrame, provider, port), limits(limits), 
                    sourceToAxis(sourceToAxis), parentToJointOrigin(parentToJointOrigin), isDynamic(false)
{

}

smurf::Joint::Joint(smurf::Frame* sourceFrame, smurf::Frame* targetFrame, const std::string& provider, 
                    const std::string& port, const std::string& driverName, base::JointLimitRange& limits, 
                    const Eigen::Affine3d& sourceToAxis, const Eigen::Affine3d& parentToJointOrigin,
                    urdf::JointSharedPtr jointModel):
                    DynamicTransformation(sourceFrame, targetFrame, provider, port), limits(limits), 
                    sourceToAxis(sourceToAxis), parentToJointOrigin(parentToJointOrigin),
                    jointModel(jointModel), isDynamic(false)
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

urdf::JointSharedPtr smurf::Joint::getJointModel() const
{
    return this -> jointModel;
};

std::pair<double, double> smurf::Joint::getPositionLimits() const
{
     return std::pair<double, double>(this->limits.min.position, this->limits.max.position);
};

std::pair<double, double> smurf::Joint::getEffortLimits() const
{
     return std::pair<double, double>(this->limits.min.effort, this->limits.max.effort);
};

std::pair<double, double> smurf::Joint::getSpeedLimits() const
{
     return std::pair<double, double>(this->limits.min.speed, this->limits.max.speed);
}

void smurf::Joint::setParamFromConfigMap(configmaps::ConfigMap configMap)
{
     isDynamic = true;

     if (configMap.hasKey("damping_const_constraint_axis1"))
          springParam.damping_const_constraint_axis1 = static_cast<double>(configMap["damping_const_constraint_axis1"]);
     else
          isDynamic = false;
     
     if (configMap.hasKey("springDamping"))
          springParam.springDamping = static_cast<double>(configMap["springDamping"]);
     else
          isDynamic = false;
     
     if (configMap.hasKey("springStiffness"))
          springParam.springStiffness = static_cast<double>(configMap["springStiffness"]);
     else
          isDynamic = false;
     
     if (configMap.hasKey("spring_const_constraint_axis1"))
          springParam.spring_const_constraint_axis1 = static_cast<double>(configMap["spring_const_constraint_axis1"]);
     else
          isDynamic = false;
}

smurf::SpringParam smurf::Joint::getSpringParam() const
{
     return springParam;
}

bool smurf::Joint::hasSpring() const 
{
     return isDynamic;
}
