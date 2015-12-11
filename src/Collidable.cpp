#include "Collidable.hpp"

smurf::Collidable::Collidable(const std::string& name, const std::string& bitmask)
{
    this->name=name;
    this->bitmask=bitmask;
}

std::string smurf::Collidable::getName()
{
    return this->name;
}

std::string smurf::Collidable::getBitmask()
{
    return this->bitmask;
}