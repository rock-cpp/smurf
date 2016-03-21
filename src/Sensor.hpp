#ifndef SENSOR_H
#define SENSOR_H

#include "Frame.hpp"
#include <configmaps/ConfigData.h>

namespace smurf
{
    
    class Sensor
    {
    public:
        Sensor(const std::string &name, const std::string &type, const std::string &taskInstanceName, Frame *inFrame);
        Sensor(const std::string &name, const std::string &type, const std::string &taskInstanceName, Frame *inFrame, configmaps::ConfigMap sensorMap);
        Sensor();
        std::string getName();
        std::string getType();
        std::string getTaskInstanceName();
        Frame * getAttachmentPoint();
        
        configmaps::ConfigMap getMap(){return this->map;}
        
    private:
        std::string name;
        std::string type;
        
        std::string taskInstanceName;
        
        Frame *attachmentPoint;
        
        configmaps::ConfigMap map;
    };
    
};

#endif // SENSOR_H
