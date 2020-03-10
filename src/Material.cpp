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

smurf::Material::Material()
{}

smurf::Material::Material(urdf::MaterialSharedPtr material)
{
    name = material->name;
    texture_filename = material->texture_filename;
    diffuseColor = material->color;
}

//TODO: add color comparison
bool  smurf::Material::operator==(const smurf::Material& other) const
{
    return other.name == name &&
           other.texture_filename == texture_filename;
           /*other.ambientColor == ambientColor &&
           other.diffuseColor == diffuseColor &&
           other.specularColor == specularColor &&
           other.shininess == shininess;*/
}

bool smurf::Material::operator!=(const smurf::Material& other) const
{
    return !operator==(other);
}

void smurf::Material::setName(std::string name) {
    this->name = name;
}

std::string smurf::Material::getName() const {
    return this->name;
}

void smurf::Material::setTextureFilename(std::string texture_filename) {
    this->texture_filename = texture_filename;
}

std::string smurf::Material::getTextureFilename() const {
    return texture_filename;
} 

void smurf::Material::setAmbientColor(urdf::Color color) {
    this->ambientColor = color;
}

urdf::Color smurf::Material::getAmbientColor() const {
    return this->ambientColor;
}

void smurf::Material::setDiffuseColor(urdf::Color color) {
    this->diffuseColor = color;
}

urdf::Color smurf::Material::getDiffuseColor() const {
    return this->diffuseColor;
}

void smurf::Material::setSpecularColor(urdf::Color color) {
    this->specularColor = color;
}

urdf::Color smurf::Material::getSpecularColor() const {
    return this->specularColor;
}

void smurf::Material::setShininess(float shininess) {
    this->shininess = shininess;
}

float smurf::Material::getShininess() const {
    return this->shininess;
}


