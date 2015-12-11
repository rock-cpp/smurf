#include "Frame.hpp"

smurf::Frame::Frame(const std::string &name, const std::vector<urdf::Visual>& visuals) :
    name(name), visuals(visuals)
{
}

smurf::Frame::Frame(const std::string& name): name(name)
{
}

smurf::Frame::Frame()
{
}

void smurf::Frame::addVisual(const urdf::Visual& visual)
{
    visuals.push_back(visual);
}

void smurf::Frame::setVisuals(const std::vector<urdf::Visual>& visuals)
{
    this->visuals = visuals;
}

void smurf::Frame::getVisuals(std::vector<urdf::Visual> & Visuals) const
{
     Visuals=this->visuals;
}

std::vector<urdf::Visual> &smurf::Frame::getVisuals()
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

/* TODO
void addCollidable(const smurf::Collidable & collidable);

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