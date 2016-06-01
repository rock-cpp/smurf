#ifndef DYNAMICTRANSFORMATION_H
#define DYNAMICTRANSFORMATION_H

#include "Transformation.hpp"

namespace smurf
{
    
    class DynamicTransformation : public Transformation
    {
    public:
        DynamicTransformation(const std::string &name, Frame *sourceFrame, Frame *targetFrame, const std::string &provider, const std::string &port);
        
        DynamicTransformation(Frame *sourceFrame, Frame *targetFrame, const std::string &provider, const std::string &port);

        const std::string &getProviderName() const
        {
            return providerName;
        };
        
        const std::string &getProviderPortName() const
        {
            return providerPortName;
        };
        
    private:
        /**
         * Name of the task instance that provides the
         * dynamic transformation
         * */
        std::string providerName;
        
        /**
         * Name of the port on the task instance, that
         * provides the transformation.
         * */
        std::string providerPortName;
    };
    
};

#endif // DYNAMICTRANSFORMATION_H
