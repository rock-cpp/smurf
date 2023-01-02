#ifndef STATICTRANSFORMATION_H
#define STATICTRANSFORMATION_H

#include "Transformation.hpp"

namespace smurf
{

    class StaticTransformation : public Transformation
    {
    public:
        StaticTransformation();
        StaticTransformation(Frame* sourceFrame, Frame* targetFrame, const Eigen::Affine3d &sourceToTarget);
        StaticTransformation(Frame* sourceFrame, Frame* targetFrame, const Eigen::Quaterniond &rotation, const Eigen::Vector3d &translation);
        StaticTransformation(const std::string &name, Frame* sourceFrame, Frame* targetFrame, const Eigen::Affine3d &sourceToTarget);
        StaticTransformation(const std::string &name, Frame* sourceFrame, Frame* targetFrame, const Eigen::Quaterniond &rotation, const Eigen::Vector3d &translation);

        const Eigen::Affine3d &getTransformation() const;

        /**Serializes the members of this class*/
        template <typename Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            throw std::runtime_error("Smurf::Motor::serialize not implemented");
        }

    private:
        /**
         * Transformation from the source frame
         * to the target frame.
         * */
        Eigen::Affine3d sourceToTarget;
    };

};

#endif // STATICTRANSFORMATION_H
