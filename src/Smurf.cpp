#include "Smurf.hpp"

smurf::Robot::Robot()
{

}

void smurf::Robot::loadFromSmurf(const std::string& path)
{

}

smurf::Transformation::Transformation(smurf::Frame* sourceFrame, smurf::Frame* targetFrame) : name(sourceFrame->getName() + "2" + targetFrame->getName()), sourceFrame(sourceFrame), targetFrame(targetFrame)
{

}



