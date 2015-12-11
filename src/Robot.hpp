#ifndef SMURF_H
#define SMURF_H

#include "StaticTransformation.hpp"
#include "DynamicTransformation.hpp"
#include "Joint.hpp"
#include "Sensor.hpp"

/*
 * TODO Do we miss this includes somewhere?
#include <base/samples/RigidBodyState.hpp>
*/

namespace smurf
{
    class Robot
    {
    public:
        
        Robot();
        
        void loadFromSmurf(const std::string &path);
        
        void loadCollidables();
        
        const std::vector<smurf::StaticTransformation *> & getStaticTransforms() const
        {
            return staticTransforms;
        };
        
        const std::vector<smurf::DynamicTransformation *> & getDynamicTransforms() const
        {
            return dynamicTransforms;
        };
        
        const std::vector<smurf::Joint *> & getJoints() const
        {
            return joints;
        };
        
        const std::vector<smurf::Sensor *> & getSensors() const
        {
            return sensors;
        };
        
        const std::vector<smurf::Frame *> & getFrames() const
        {
            return availableFrames;
        };
        
        const Frame* getRootFrame() const
        {
            return rootFrame;
        };
        
    protected:
        
        Frame *getFrameByName(const std::string &name);
        
        Frame *rootFrame;
        boost::shared_ptr<urdf::ModelInterface> model;
        
        std::vector<smurf::Frame *> availableFrames;
        std::vector<smurf::StaticTransformation *> staticTransforms;
        std::vector<smurf::DynamicTransformation *> dynamicTransforms;
        std::vector<smurf::Joint *> joints;
        std::vector<smurf::Sensor *> sensors;
    };
};
#endif // SMURF_H
