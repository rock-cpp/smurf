#include "Collidable.hpp"
#include "utils.hpp"

#include <base-logging/Logging.hpp>

smurf::Collidable::Collidable()
{
}

smurf::Collidable::Collidable(const urdf::Collision &collision, const ContactParams contactParams)
{
    name = collision.name;
    origin = utils::convertPose(collision.origin);
    geometry.reset(utils::createGeometry(collision.geometry));
    this->contactParams = contactParams;

    // TODO: groupID?
}

smurf::Collidable::Collidable(configmaps::ConfigMap &configMap)
{
    name = "";
    // TODO: check if config map contains the required parameter
    if (!configMap.hasKey("name"))
        LOG_ERROR_S << "There is no name key in config map. The name will be empty.";
    else
        name = configMap["name"].toString();

    if (!configMap.hasKey("position") || !configMap["position"].hasKey("x")
                                      || !configMap["position"].hasKey("y")
                                      || !configMap["position"].hasKey("z"))
        LOG_ERROR_S << "The position key is not set or set wrong in config map, therefore the position will be set to zero.";
    else
        origin.position = base::Position(configMap["position"]["x"],
                                         configMap["position"]["y"],
                                         configMap["position"]["z"]);

    if (!configMap.hasKey("rotation") || !configMap["rotation"].hasKey("w")
                                      || !configMap["rotation"].hasKey("x")
                                      || !configMap["rotation"].hasKey("y")
                                      || !configMap["rotation"].hasKey("z"))
        LOG_ERROR_S << "The rotation key is not set or set wrong in config map, therefore rotation will be set to zero.";
    else
        origin.orientation = base::Orientation(configMap["rotation"]["w"],
                                               configMap["rotation"]["x"],
                                               configMap["rotation"]["y"],
                                               configMap["rotation"]["z"]);

    geometry.reset(utils::createGeometry(configMap));

    // TODO: groupID?
    // TODO: contact Param?
}

configmaps::ConfigMap smurf::Collidable::getConfigMap() const
{
    configmaps::ConfigMap configMap;
    if (geometry == nullptr)
        LOG_ERROR_S << "No geometry was set for the collision with the name " << name;
    else
        configMap = geometry->getConfigMap();

    configMap["name"] = name;
    configMap["position"]["x"] = origin.position.x();
    configMap["position"]["y"] = origin.position.y();
    configMap["position"]["z"] = origin.position.z();
    configMap["rotation"]["w"] = origin.orientation.w();
    configMap["rotation"]["x"] = origin.orientation.x();
    configMap["rotation"]["y"] = origin.orientation.y();
    configMap["rotation"]["z"] = origin.orientation.z();

    // TODO: Contact Params
    // TODO: groupID?

    return configMap;
}

// TODO. the comparison operator
// add origin comparison
bool smurf::Collidable::operator==(const smurf::Collidable &other) const
{
    return this->geometry == other.geometry &&
           this->name == other.name;
}

bool smurf::Collidable::operator!=(const smurf::Collidable &other) const
{
    return !operator==(other);
}
