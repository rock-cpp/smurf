#ifndef SENSOR_H
#define SENSOR_H

#include "Frame.hpp"

namespace smurf
{
    
    class Sensor
    {
    public:
        Sensor(const std::string &name, const std::string &type, const std::string &taskInstanceName, Frame *inFrame);
        Sensor();
        std::string getName();
        std::string getType();
        std::string getTaskInstanceName();
        Frame * getAttachmentPoint();
    private:
        std::string name;
        std::string type;
        
        std::string taskInstanceName;
        
        Frame *attachmentPoint;
    };
    
};

#endif // SENSOR_H
