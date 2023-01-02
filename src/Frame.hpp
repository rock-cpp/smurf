#ifndef FRAME_H
#define FRAME_H

#include <urdf_model/model.h>
#include "Collidable.hpp"
#include "Inertial.hpp"
#include "Visual.hpp"

namespace smurf
{

    class Frame
    {

    public:
        Frame(const std::string &name, const std::vector<smurf::Visual>& visuals);
        Frame(const std::string &name);
        Frame();

        const std::string getName() const
        {
            return name;
        };

        void addVisual(const smurf::Visual& visual);
        void setVisuals(const std::vector<smurf::Visual>& visuals);
        void getVisuals(std::vector<smurf::Visual>& Visuals) const;
        std::vector<smurf::Visual>& getVisuals();
        /**
         * Collisions are the objects defined for collision detection in the URDF model.
         */
        std::vector< urdf::Collision >& getCollisions();
        void addCollision(const urdf::Collision& collision);
        /**
         * Collidables are the objects defined in the SMURF model that extend the information from the URDF.
         */
        std::vector<smurf::Collidable> &getCollidables();
        void getCollidables(std::vector<smurf::Collidable> &Collidables) const;

        void addCollidable(const smurf::Collidable & collidable);

        void setInertial(const smurf::Inertial& inertialM)
        {
            this->inertial = inertialM;
            this->hasInertial = true;
        };

        smurf::Inertial getInertial()
        {
            return this->inertial;
        };

        void setGroupId(int id)
        {
            this->groupId = id;
        };

        int getGroupId() const
        {
            return this->groupId;
        };

        bool getHasInertial() const
        {
            return this->hasInertial;
        }

        /*
        void setCollidables(const std::vector<smurf::Collidable>& collidables);
        */

        /**Grants access to boost serialization */
        friend class boost::serialization::access;

        /**Serializes the members of this class*/
        template <typename Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            throw std::runtime_error("Smurf::Frame::serialize not implemented");
        }

    private:
        ///Name of the frame
        std::string name;

        ///Collision objects inside the frame
        std::vector<smurf::Collidable> collidables;
        // TODO can we just use the urdf::Collision objects? No this breaks on execution for the simulator but I guess it can be needed for the collision detection outside the simulator, therefore I put it as a separate structure.
        std::vector<urdf::Collision> collisions;

        ///Visuals that can be displayed inside the frame
        std::vector<smurf::Visual> visuals;

        bool hasInertial;

        smurf::Inertial inertial;
        ///TODO add additional data in map or whatever

        int groupId;

    };

};
#endif // FRAME_H
