#ifndef COLLIDABLE_H
#define COLLIDABLE_H

#include <string>
#include <urdf_model/model.h>
#include "ContactParams.hpp"

namespace smurf{
    
    class Collidable
    {
    public:
        Collidable(const std::string& name, const ContactParams contact_params, const urdf::Collision& collision);
        std::string getName() const 
        { 
            return this->name; 
        }
        urdf::Collision getCollision() const 
        { 
            return this->collision; 
        }
        void setGroupId(const int id) 
        { 
            this->groupId = id; 
        }
        int getGroupId() const 
        { 
            return this->groupId; 
        }
        ContactParams getContactParams() const
        {
            return this->contact_params;
        }

        bool operator==(const Collidable& other) const;
        bool operator!=(const Collidable& other) const;
    private:
        std::string name;
        urdf::Collision collision;
        int groupId;
        ContactParams contact_params;
    };
    
};

#endif // COLLIDABLE_H
