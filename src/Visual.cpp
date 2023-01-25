//
// Copyright (c) 2015, Deutsches Forschungszentrum für Künstliche Intelligenz GmbH.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

#include "Visual.hpp"

#include <base-logging/Logging.hpp>

smurf::Visual::Visual()
{
}

smurf::Visual::Visual(const urdf::Visual &urdfVisual)
{
    name = urdfVisual.name;
    geometry.reset(utils::createGeometry(urdfVisual.geometry));

    // if visual has material
    if (urdfVisual.material)
        material = Material(urdfVisual.material);

    origin.position = base::Position(urdfVisual.origin.position.x,
                                     urdfVisual.origin.position.y,
                                     urdfVisual.origin.position.z);

    origin.orientation = base::Orientation(urdfVisual.origin.rotation.w,
                                           urdfVisual.origin.rotation.x,
                                           urdfVisual.origin.rotation.y,
                                           urdfVisual.origin.rotation.z);


}

smurf::Visual::Visual(configmaps::ConfigMap &configMap)
{
    name = "visual";
    // TODO: check if config map contains the required parameter
    if (!configMap.hasKey("name"))
        LOG_ERROR_S << "[Visual(configmaps::ConfigMap& configMap)] There is no name key in config map. The name will be set to default 'visual'.";
    else
        name = configMap["name"].toString();

    geometry.reset(utils::createGeometry(configMap));

    if (!configMap.hasKey("position"))
        LOG_ERROR_S << "[Visual(configmaps::ConfigMap& configMap)] There is no position key in config map. The position will be set to zero.";
    else
        // TODO: do we need to check x,y,z parameter?
        origin.position = base::Position(configMap["position"]["x"],
                                         configMap["position"]["y"],
                                         configMap["position"]["z"]);

    if (!configMap.hasKey("rotation"))
        LOG_ERROR_S << "[Visual(configmaps::ConfigMap& configMap)] There is no rotation key in config map. The rotation will be set to zero.";
    else
        // TODO: do we need to check x,y,z parameter?
        origin.orientation = base::Orientation(configMap["rotation"]["w"],
                                               configMap["rotation"]["x"],
                                               configMap["rotation"]["y"],
                                               configMap["rotation"]["z"]);

    // TODO: set material
}

// FIX. the comparison operator
// add origin comparison
bool smurf::Visual::operator==(const smurf::Visual &other) const
{
    return other.geometry == geometry &&
           other.material == material &&
           other.name == name;
}

bool smurf::Visual::operator!=(const smurf::Visual &other) const
{
    return !operator==(other);
}

void smurf::Visual::setMaterial(smurf::Material material)
{
    this->material = material;
}

smurf::Material smurf::Visual::getMaterial() const
{
    return this->material;
}

configmaps::ConfigMap smurf::Visual::getConfigMap() const
{
    configmaps::ConfigMap configMap = geometry->getConfigMap();
    configMap["name"] = name;
    configMap["position"]["x"] = origin.position.x();
    configMap["position"]["y"] = origin.position.y();
    configMap["position"]["z"] = origin.position.z();
    configMap["rotation"]["w"] = origin.orientation.w();
    configMap["rotation"]["x"] = origin.orientation.x();
    configMap["rotation"]["y"] = origin.orientation.y();
    configMap["rotation"]["z"] = origin.orientation.z();

    configMap["material"] = material.getConfigMap();

    return configMap;
}
