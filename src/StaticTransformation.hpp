#ifndef STATICTRANSFORMATION_H
#define STATICTRANSFORMATION_H

#include "Transformation.hpp"

namespace smurf
{
    
    class StaticTransformation : public Transformation
    {
    public:
        StaticTransformation(Frame* sourceFrame, Frame* targetFrame, const Eigen::Affine3d &sourceToTarget);
        StaticTransformation(Frame* sourceFrame, Frame* targetFrame, const Eigen::Quaterniond &rotation, const Eigen::Vector3d &translation);
        
        const Eigen::Affine3d &getTransformation() const;
        
    private:
        /**
         * Transformation from the source frame
         * to the target frame.
         * */
        Eigen::Affine3d sourceToTarget;
    };
    
};

#endif // STATICTRANSFORMATION_H
