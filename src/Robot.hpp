#ifndef SMURF_H
#define SMURF_H

#include "StaticTransformation.hpp"
#include "DynamicTransformation.hpp"
#include "Joint.hpp"
#include "Sensor.hpp"
#include <configmaps/ConfigData.h>

#include <mars/interfaces/contact_params.h>

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
        
        /**
         * Loads the URDF collision objects
         * 
         */
        void loadCollisions();
        
        void loadInertials();
        
        void loadJoints();
        
        const mars::interfaces::contact_params getContactParams(const std::string& collisionName, const std::string& linkName);

        /**
         * Loads the SMURF Collidable objects
         */
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
        configmaps::ConfigMap smurfMap;
        
        std::vector<smurf::Frame *> availableFrames;
        std::vector<smurf::StaticTransformation *> staticTransforms;
        std::vector<smurf::DynamicTransformation *> dynamicTransforms;
        std::vector<smurf::Joint *> joints;
        std::vector<smurf::Sensor *> sensors;
        
        const bool debug = false;
    };
};
#endif // SMURF_H