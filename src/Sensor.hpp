#ifndef SENSOR_H
#define SENSOR_H

#include "Frame.hpp"
#include <configmaps/ConfigData.h>

namespace smurf
{

    class Sensor
    {
    public:
        Sensor();
        Sensor(const std::string &name, const std::string &type, const std::string &taskInstanceName, Frame *inFrame);
        Sensor(const std::string &name, const std::string &type, const std::string &taskInstanceName, Frame *inFrame, configmaps::ConfigMap sensorMap);
        std::string getName() const;
        std::string getType() const;
        std::string getTaskInstanceName();
        Frame * getAttachmentPoint();
        std::string getJointName() const;

        configmaps::ConfigMap getMap() const {return this->map;}

        void setJointName(std::string jointName);

        /**Serializes the members of this class*/
        template <typename Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            throw std::runtime_error("Smurf::Sensor::serialize not implemented");
        }

    private:
        std::string name;
        std::string type;

        std::string taskInstanceName;

        Frame *attachmentPoint;

        configmaps::ConfigMap map;

        std::string jointName;
    };

};

#endif // SENSOR_H
