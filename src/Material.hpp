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

#ifndef MATERIAL_H
#define MATERIAL_H
#include <urdf_model/types.h>
#include <urdf_model/link.h>
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>

#include <configmaps/ConfigMap.hpp>

namespace smurf
{

    struct Color
    {
        Color();
        Color(const urdf::Color &color);
        Color(configmaps::ConfigMap &configMap);

        double r;
        double g;
        double b;
        double a;

        Color& operator=(const urdf::Color &color);

        configmaps::ConfigMap getConfigMap() const;
    };

    /**Extended verson of material.
     * The urdf::Material contains only name of material, texture_filename and diffuse color.
     * Therefore, smurf::Material extends the urdf::Material by ambientColor, specularColor and shininess
    */
    struct Material
    {
        Material();
        Material(urdf::MaterialSharedPtr material);
        Material(configmaps::ConfigMap &configMap);

        std::string name;
        std::string textureFilename;

        Color ambientColor;
        Color diffuseColor;
        Color specularColor;
        double shininess;
        configmaps::ConfigMap map;

        bool operator==(const Material& other) const;
        bool operator!=(const Material& other) const;

        configmaps::ConfigMap getConfigMap() const;

        /**Grants access to boost serialization */
        friend class boost::serialization::access;

        /**Serializes the members of this class*/
        template <typename Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            throw std::runtime_error("Smurf::Visual::serialize not implemented");
        }
    };
}

#endif // MATERIAL_H
