#ifndef TRANSFORMATION_H
#define TRANSFORMATION_H

#include "Frame.hpp"
#include <base/Eigen.hpp>

namespace smurf
{
    
    class Transformation
    {
    public:
        
        Transformation(Frame *sourceFrame, Frame *targetFrame);
        
        const Frame &getSourceFrame() const
        {
            return *sourceFrame;
        }
        
        const Frame &getTargetFrame() const
        {
            return *targetFrame;
        }
        
        const std::string &getName() const
        {
            return name;
        }
        
    private:
        
        /**
         * Name of the Transformation, defaults to
         * "<sourceFrameName>2<TargetFrameName>"
         * */
        std::string name;
        
        Frame *sourceFrame;
        Frame *targetFrame;
    };
    
};

#endif // TRANSFORMATION_H