#include "Collidable.hpp"

smurf::Collidable::Collidable(const std::string& name, const mars::interfaces::contact_params contact_params, const urdf::Collision& collision)
{
    this->name=name;
    this->collision=collision;
    this->contact_params = contact_params;
}