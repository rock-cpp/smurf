#include "Sensor.hpp"

smurf::Sensor::Sensor()
{
}

smurf::Sensor::Sensor(const std::string &name, const std::string &type, const std::string &taskInstanceName, Frame *inFrame)
{
    this->name=name;
    this->type=type;
    this->taskInstanceName=taskInstanceName;
    this->attachmentPoint=inFrame;
}

std::string smurf::Sensor::getName()
{
    return this->name;
}

std::string smurf::Sensor::getType()
{
    return this->type;
}

smurf::Frame * smurf::Sensor::getAttachmentPoint()
{
    return this->attachmentPoint;
}

std::string smurf::Sensor::getTaskInstanceName()
{
    return this->taskInstanceName;
}
