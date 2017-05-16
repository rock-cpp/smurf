#include "Collidable.hpp"

smurf::Collidable::Collidable(const std::string& name, const ContactParams contact_params, const urdf::Collision& collision)
{
    this->name=name;
    this->collision=collision;
    this->contact_params = contact_params;
}

bool  smurf::Collidable::operator==(const smurf::Collidable& other) const
{
    return this->collision.geometry == other.collision.geometry &&
           this->name == other.name;
}

bool smurf::Collidable::operator!=(const smurf::Collidable& other) const
{
    return !operator==(other);
}
