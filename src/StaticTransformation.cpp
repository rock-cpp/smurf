#include "StaticTransformation.hpp"

smurf::StaticTransformation::StaticTransformation(const std::string &name, smurf::Frame* sourceFrame, smurf::Frame* targetFrame, const Eigen::Affine3d& sourceToTarget): 
    Transformation(name, sourceFrame, targetFrame), sourceToTarget(sourceToTarget)
{

}

smurf::StaticTransformation::StaticTransformation(const std::string &name, smurf::Frame* sourceFrame, smurf::Frame* targetFrame, const Eigen::Quaterniond& rotation, const Eigen::Vector3d& translation): 
    Transformation(name, sourceFrame, targetFrame)
{
    sourceToTarget.setIdentity();
    sourceToTarget.rotate(rotation);
    sourceToTarget.translation() = translation;
}

const Eigen::Affine3d& smurf::StaticTransformation::getTransformation() const
{
    return sourceToTarget;
}
