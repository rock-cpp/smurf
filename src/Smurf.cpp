#include "Smurf.hpp"

smurf::Frame::Frame(const std::string& name): name(name)
{

}


smurf::StaticTransformation::StaticTransformation(smurf::Frame* sourceFrame, smurf::Frame* targetFrame, const Eigen::Affine3d& sourceToTarget): 
    Transformation(sourceFrame, targetFrame), sourceToTarget(sourceToTarget)
{

}

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

void smurf::Robot::loadFromSmurf(const std::string& path)
{

}

smurf::Transformation::Transformation(smurf::Frame* sourceFrame, smurf::Frame* targetFrame) : name(sourceFrame->getName() + "2" + targetFrame->getName()), sourceFrame(sourceFrame), targetFrame(targetFrame)
{

}



