#include "DynamicTransformation.hpp"


smurf::DynamicTransformation::DynamicTransformation(const std::string& name, smurf::Frame* sourceFrame, smurf::Frame* targetFrame, const std::string& provider, const std::string& port): 
    Transformation(name, sourceFrame, targetFrame), providerName(provider), providerPortName(port)
{
}

smurf::DynamicTransformation::DynamicTransformation(smurf::Frame* sourceFrame, smurf::Frame* targetFrame, const std::string& provider, const std::string& port): 
    Transformation(sourceFrame, targetFrame), providerName(provider), providerPortName(port)
{
}

smurf::DynamicTransformation::DynamicTransformation(const std::string &name, Frame *sourceFrame, Frame *targetFrame):
    Transformation(sourceFrame, targetFrame), providerName(""), providerPortName("")
{
}
