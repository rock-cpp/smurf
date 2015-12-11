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

std::string smurf::Sensor::getname()
{
    return this->name;
}

std::string smurf::Sensor::gettype()
{
    return this->type;
}

smurf::Frame * smurf::Sensor::getattachmentPoint()
{
    return this->attachmentPoint;
}

std::string smurf::Sensor::gettaskInstanceName()
{
    return this->taskInstanceName;
}




