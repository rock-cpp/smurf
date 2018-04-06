#include "Frame.hpp"

smurf::Frame::Frame(const std::string &name, const std::vector<smurf::Visual>& visuals) :
    name(name), visuals(visuals), hasInertial(false)
{
}

smurf::Frame::Frame(const std::string& name): name(name), hasInertial(false)
{
}

smurf::Frame::Frame(): hasInertial(false)
{
}

void smurf::Frame::addVisual(const smurf::Visual& visual)
{
    visuals.push_back(visual);
}

void smurf::Frame::setVisuals(const std::vector<smurf::Visual>& visuals)
{
    this->visuals = visuals;
}

void smurf::Frame::getVisuals(std::vector<smurf::Visual> & Visuals) const
{
     Visuals=this->visuals;
}

std::vector<smurf::Visual> &smurf::Frame::getVisuals()
{
    return this->visuals;
}

void smurf::Frame::addCollision(const urdf::Collision& collision)
{
    collisions.push_back(collision);
}

std::vector<urdf::Collision> &smurf::Frame::getCollisions()
{
    return this -> collisions;
}

void smurf::Frame::addCollidable(const smurf::Collidable& collidable)
{
    collidables.push_back(collidable);
}

/* TODO
void setCollidables(const std::vector<smurf::Collidable>& collidables);
*/

void smurf::Frame::getCollidables(std::vector<smurf::Collidable> &collidables) const
{
    collidables=this->collidables;
}

std::vector<smurf::Collidable> &smurf::Frame::getCollidables()
{
    return this->collidables;
}