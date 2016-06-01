#include "TranslationalJoint.hpp"

smurf::TranslationalJoint::TranslationalJoint(const std::string &name, smurf::Frame* sourceFrame, smurf::Frame* targetFrame, const std::string& provider, 
                                              const std::string& port, const std::string& driverName, base::JointLimitRange& limits, 
                                              const Eigen::Affine3d& sourceToAxis, const Eigen::Vector3d& translationAxis): 
                                              Joint(name, sourceFrame, targetFrame, provider, port, driverName, limits, sourceToAxis), 
                                              translationAxis(translationAxis){}
                                              
smurf::TranslationalJoint::TranslationalJoint(const std::string &name, smurf::Frame* sourceFrame, smurf::Frame* targetFrame, const std::string& provider, 
                                              const std::string& port, const std::string& driverName, base::JointLimitRange& limits, 
                                              const Eigen::Affine3d& sourceToAxis, const Eigen::Vector3d& translationAxis, 
                                              const Eigen::Affine3d& parentToJointOrigin, boost::shared_ptr<urdf::Joint> jointModel): 
                                              Joint(name, sourceFrame, targetFrame, provider, port, driverName, limits, sourceToAxis, parentToJointOrigin, jointModel), 
                                              translationAxis(translationAxis)
                                              {}

smurf::TranslationalJoint::TranslationalJoint(smurf::Frame* sourceFrame, smurf::Frame* targetFrame, const std::string& provider, 
                                              const std::string& port, const std::string& driverName, base::JointLimitRange& limits, 
                                              const Eigen::Affine3d& sourceToAxis, const Eigen::Vector3d& translationAxis): 
                                              Joint(sourceFrame, targetFrame, provider, port, driverName, limits, sourceToAxis), 
                                              translationAxis(translationAxis){}
                                              
smurf::TranslationalJoint::TranslationalJoint(smurf::Frame* sourceFrame, smurf::Frame* targetFrame, const std::string& provider, 
                                              const std::string& port, const std::string& driverName, base::JointLimitRange& limits, 
                                              const Eigen::Affine3d& sourceToAxis, const Eigen::Vector3d& translationAxis, 
                                              const Eigen::Affine3d& parentToJointOrigin, boost::shared_ptr<urdf::Joint> jointModel): 
                                              Joint(sourceFrame, targetFrame, provider, port, driverName, limits, sourceToAxis, parentToJointOrigin, jointModel), 
                                              translationAxis(translationAxis)
                                              {}
