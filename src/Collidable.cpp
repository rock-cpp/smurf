#include "Collidable.hpp"

smurf::Collidable::Collidable(const std::string& name, const int& bitmask, const urdf::Collision& collision)
{
    this->name=name;
    this->bitmask=bitmask;
    this->collision=collision;
}