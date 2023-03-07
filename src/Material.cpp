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

#include "Material.hpp"

#include <base-logging/Logging.hpp>

smurf::Color::Color() {}

smurf::Color::Color(const urdf::Color &color) : r(color.r), g(color.g), b(color.b), a(color.a) {}

smurf::Color::Color(configmaps::ConfigMap &configMap)
{
    if (!configMap.hasKey("r") || !configMap.hasKey("g") || !configMap.hasKey("b"))
        LOG_ERROR_S << "The config map for color was set uncorrectly, check the keys.";
    else {
        r = configMap["r"];
        g = configMap["g"];
        b = configMap["b"];
        if (!configMap.hasKey("a"))
            a = 0.0;        // TODO: alpha 0 or 1.0
        else
            a = configMap["a"];
    }
}

smurf::Color& smurf::Color::operator=(const urdf::Color &color)
{
    r = color.r;
    g = color.g;
    b = color.b;
    a = color.a;
    return *this;
}

configmaps::ConfigMap smurf::Color::getConfigMap() const
{
    configmaps::ConfigMap configMap;
    configMap["r"] = r;
    configMap["g"] = g;
    configMap["b"] = b;
    configMap["a"] = a;
    return configMap;
}

smurf::Material::Material() {
}

smurf::Material::Material(urdf::MaterialSharedPtr material)
{
    name = material->name;
    textureFilename = material->texture_filename;
    diffuseColor = material->color;
}

smurf::Material::Material(configmaps::ConfigMap &configMap)
{
    map = configMap;
    //TODO: implement material from config map
    name = "";
    // TODO: check if config map contains the required parameter
    if (!configMap.hasKey("name"))
        LOG_ERROR_S << "There is no name key in the config map. The name will be empty.";
    else
        name = configMap["name"].toString();

    if (!configMap.hasKey("diffuseColor"))
        LOG_WARN_S << "There is no diffuse color in the config map for the material '" << name << "'";
    else
        diffuseColor = Color(configMap["diffuseColor"]);

    if (!configMap.hasKey("ambientColor"))
        LOG_WARN_S << "There is no ambient color in the config map for the material '" << name << "'";
    else
        ambientColor = Color(configMap["ambientColor"]);

    if (!configMap.hasKey("specularColor"))
        LOG_WARN_S << "There is no specular color in the config map for the material '" << name << "'";
    else
        specularColor = Color(configMap["specularColor"]);

    if (!configMap.hasKey("shininess"))
        LOG_WARN_S << "There is no shininess in the config map for the material '" << name << "'";
    else
        shininess = configMap["shininess"];

    // TODO: is it textureFilename or textureName
    // what is stored here
    if (!configMap.hasKey("texturename"))
        LOG_WARN_S << "There is no texturename in the config map for the material '" << name << "'";
    else
        textureFilename = configMap["texturename"].toString();
}

configmaps::ConfigMap smurf::Material::getConfigMap() const
{
    configmaps::ConfigMap configMap = map;
    configMap["name"] = name;
    configMap["diffuseColor"] = diffuseColor.getConfigMap();
    configMap["ambientColor"] = ambientColor.getConfigMap();
    configMap["specularColor"] = specularColor.getConfigMap();
    configMap["shininess"] = shininess;
    configMap["texturename"] = textureFilename;
    return configMap;
}

// TODO: add color comparison
bool smurf::Material::operator==(const smurf::Material &other) const
{
    return other.name == name &&
           other.textureFilename == textureFilename;
    /*other.ambientColor == ambientColor &&
    other.diffuseColor == diffuseColor &&
    other.specularColor == specularColor &&
    other.shininess == shininess;*/
}

bool smurf::Material::operator!=(const smurf::Material &other) const
{
    return !operator==(other);
}
