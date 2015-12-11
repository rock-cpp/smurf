#include "DynamicTransformation.hpp"


smurf::DynamicTransformation::DynamicTransformation(smurf::Frame* sourceFrame, smurf::Frame* targetFrame, const std::string& provider, const std::string& port): 
    Transformation(sourceFrame, targetFrame), providerName(provider), providerPortName(port)
{

}