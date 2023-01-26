#ifndef COLLIDABLE_H
#define COLLIDABLE_H

#include <string>
#include <urdf_model/model.h>
#include "ContactParams.hpp"
#include "Geometry.hpp"

#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>

namespace smurf{

    struct Collidable
    {
        Collidable();
        Collidable(const urdf::Collision& collision, const ContactParams contactParams);
        Collidable(configmaps::ConfigMap& configMap);

        std::string name;
        base::Pose origin;
        std::shared_ptr<Geometry> geometry;
        ContactParams contactParams;
        // TODO: check where do we need groupId
        int groupId;

        bool operator==(const Collidable& other) const;
        bool operator!=(const Collidable& other) const;

        configmaps::ConfigMap getConfigMap() const;

        /**Grants access to boost serialization */
        friend class boost::serialization::access;

        /**Serializes the members of this class*/
        template <typename Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            throw std::runtime_error("smurf::Collidable::serialize not implemented");
        }
    };

};

#endif // COLLIDABLE_H
