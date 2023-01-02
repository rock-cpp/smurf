#include "Transformation.hpp"

smurf::Transformation::Transformation()
{}

smurf::Transformation::Transformation(const std::string &name, smurf::Frame* sourceFrame, smurf::Frame* targetFrame) : name(name), sourceFrame(sourceFrame), targetFrame(targetFrame)
{

}

smurf::Transformation::Transformation(smurf::Frame* sourceFrame, smurf::Frame* targetFrame) : name(sourceFrame->getName() + "2" + targetFrame->getName()), sourceFrame(sourceFrame), targetFrame(targetFrame)
{

}
