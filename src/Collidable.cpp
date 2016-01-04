#include "Collidable.hpp"

smurf::Collidable::Collidable(const std::string& name, const int& bitmask, const urdf::Collision& collision)
{
    this->name=name;
    this->bitmask=bitmask;
    this->collision=collision;
}

urdf::Collision smurf::Collidable::getCollision()
{
    return this->collision;
}

std::string smurf::Collidable::getName()
{
    return this->name;
}

int smurf::Collidable::getBitmask()
{
    return this->bitmask;
}