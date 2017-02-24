#ifndef SMURF_H
#define SMURF_H

#include "StaticTransformation.hpp"
#include "DynamicTransformation.hpp"
#include "Joint.hpp"
#include "Sensor.hpp"
#include "Motor.hpp"
#include "ContactParams.hpp"
#include <configmaps/ConfigData.h>
#include <urdf_world/types.h>


namespace configmaps
{
    class ConfigMap;
};

namespace smurf
{
    class Robot
    {
    public:
        
        Robot();
        
        ~Robot();
        
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
        
        const std::vector<smurf::Motor *> & getMotors() const
        {
            return motors;
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

        const ContactParams getContactParams(const std::string& collisionName, const std::string& linkName);

        /**
         * Loads the SMURF Collidable objects in their correspondent frames
         * 
         * The smurf collidable contains the urdf collision object. Therefore 
         * if loadCollidables is executed, loadCollisions in no needed.
         */
        void loadCollidables();
        
        /**
         * Loads the URDF collision objects in their correspondent frames
         */
        void loadCollisions();
        
        /**
         * Loads the SMURF Inertial objects in their correspondent frames
         * 
         * The smurf inertial object contains the urdf inertial object.
         */
        void loadInertials();
        
        /**
         * Loads the different joints of the model in the correspondent 
         * frames.
         * 
         * - Fixed joints are loaded as staticTransforms
         * - Floating, revolute and prismatic joints as dynamicTransforms
         * and as Joints.
         */
        void loadJoints();
        
        void loadMotors();
        
        void loadSensors();
        
        void loadVisuals();

        void loadFrames(urdf::ModelInterfaceSharedPtr model);
        
        /**
         * Loads all the information from the Smurf model in the Robot 
         * object.
         * 
         * - Creates the frames (from the links and from the visuals)
         * - Loads the joints
         * - Loads the sensors
         * - Loads the collidables
         * - Loads the inertials
         */
        void loadFromSmurf(const std::string &path);
        
    protected:
        
        Frame *getFrameByName(const std::string &name);

        configmaps::ConfigMap getAnnotations(const urdf::JointSharedPtr &joint);
        
        Frame *rootFrame;
        urdf::ModelInterfaceSharedPtr model;
        configmaps::ConfigMap *smurfMap;
        
        std::vector<smurf::Frame *> availableFrames;
        std::vector<smurf::StaticTransformation *> staticTransforms;
        std::vector<smurf::DynamicTransformation *> dynamicTransforms;
        std::vector<smurf::Joint *> joints;
        std::vector<smurf::Sensor *> sensors;
        std::vector<smurf::Motor *> motors;
        
        const bool debug = false;
    };
};
#endif // SMURF_H
