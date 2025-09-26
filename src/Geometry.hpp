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

#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include <algorithm>

#include <base/Pose.hpp>
#include <configmaps/ConfigMap.hpp>
#include <urdf_model/link.h>
#include <base-logging/Logging.hpp>

namespace smurf
{
    struct Geometry
    {
        // TODO: add TERRAIN and HEIGHTMAP type
        enum GeometryType
        {
            BOX,
            CAPSULE,
            CYLINDER,
            MESH,
            PLANE,
            SPHERE,
            UNKNOWN_TYPE
        };

        GeometryType type;

        Geometry() : type(UNKNOWN_TYPE) {}
        Geometry(GeometryType type) : type(type) {}
        virtual ~Geometry(void) {}

        virtual configmaps::ConfigMap getConfigMap() const
        {
            configmaps::ConfigMap configMap;
            configMap["type"] = toString(type);
            return configMap;
        }

        static GeometryType fromString(std::string geometryType)
        {
            std::transform(geometryType.begin(), geometryType.end(), geometryType.begin(), [](unsigned char c)
                           { return std::tolower(c); });

            if (geometryType == "box")
                return BOX;
            else if (geometryType == "capsule")
                return CAPSULE;
            else if (geometryType == "cylinder")
                return CYLINDER;
            else if (geometryType == "mesh")
                return MESH;
            else if (geometryType == "plane")
                return PLANE;
            else if (geometryType == "sphere")
                return SPHERE;
            else
                return UNKNOWN_TYPE;
        }

        static std::string toString(GeometryType geometryType)
        {
            switch (geometryType)
            {
            case BOX:
                return "box";
            case CAPSULE:
                return "capsule";
            case CYLINDER:
                return "cylinder";
            case MESH:
                return "mesh";
            case PLANE:
                return "plane";
            case SPHERE:
                return "sphere";
            default:
                return "unknown_type";
            }
        }

        bool isType(std::string geometryType) const
        {
            std::string typeString = toString(type);
            std::transform(geometryType.begin(), geometryType.end(), geometryType.begin(), [](unsigned char c)
                           { return std::tolower(c); });

            if (typeString == geometryType)
                return true;
            else
                return false;
        }
    };

    struct Box : public Geometry
    {
        Box() : Box(base::Vector3d::Zero()) {}
        Box(base::Vector3d size) : Geometry(Geometry::BOX), size(size) {}
        Box(configmaps::ConfigMap &configMap) : Geometry(Geometry::BOX)
        {
            if (configMap.hasKey("type") && isType(configMap["type"].toString()))
            {
                if (configMap.hasKey("size") && configMap["size"].hasKey("x") && configMap["size"].hasKey("y") && configMap["size"].hasKey("z"))
                {
                    size.x() = configMap["size"]["x"];
                    size.y() = configMap["size"]["y"];
                    size.z() = configMap["size"]["z"];
                }
                else
                {
                    LOG_ERROR_S << "The config map has no or wrong structure of the key 'size'";
                    size = base::Vector3d::Zero();
                }
            }
            else
            {
                LOG_ERROR_S << "The config map has no key 'type' or wrong type value";
                size = base::Vector3d::Zero();
            }
        }
        Box(urdf::BoxSharedPtr urdfGeometry) : Geometry(Geometry::BOX)
        {
            if (urdfGeometry != nullptr)
            {
                size.x() = urdfGeometry->dim.x;
                size.y() = urdfGeometry->dim.y;
                size.z() = urdfGeometry->dim.z;
            }
            else
            {
                size = base::Vector3d::Zero();
            }
        }

        base::Vector3d size;

        configmaps::ConfigMap getConfigMap() const override
        {
            configmaps::ConfigMap configMap = Geometry::getConfigMap();
            configMap["type"] = toString(type);
            configMap["size"]["x"] = size.x();
            configMap["size"]["y"] = size.y();
            configMap["size"]["z"] = size.z();
            return configMap;
        }
    };

    struct Capsule : public Geometry
    {
        Capsule() : Capsule(0., 0.) {}
        Capsule(double radius, double length) : Geometry(Geometry::CAPSULE), radius(radius), length(length) {}
        Capsule(configmaps::ConfigMap &configMap) : Geometry(Geometry::CAPSULE)
        {
            if (configMap.hasKey("type") && isType(configMap["type"].toString()))
            {
                if (configMap.hasKey("radius") && configMap.hasKey("length"))
                {
                    radius = configMap["radius"];
                    length = configMap["length"];
                }
                else
                {
                    LOG_ERROR_S << "The config map has no key 'radius' and/or 'length'";
                    radius = 0.;
                    length = 0.;
                }
            }
            else
            {
                LOG_ERROR_S << "The config map has no key 'type' or wrong type value";
                radius = 0.;
                length = 0.;
            }
        }

        double radius;
        double length;

        // TODO: is extend not mars specific config map parameter?
        // can we use radius and length?
        configmaps::ConfigMap getConfigMap() const override
        {
            configmaps::ConfigMap configMap = Geometry::getConfigMap();
            configMap["type"] = toString(type);
            configMap["radius"] = radius;
            configMap["length"] = length;
            return configMap;
        }
    };

    struct Cylinder : public Geometry
    {
        Cylinder() : Cylinder(0., 0.) {}
        Cylinder(double radius, double length) : Geometry(Geometry::CYLINDER), radius(radius), length(length) {}
        Cylinder(configmaps::ConfigMap &configMap) : Geometry(Geometry::CYLINDER)
        {
            if (configMap.hasKey("type") && isType(configMap["type"].toString()))
            {
                if (configMap.hasKey("radius") && configMap.hasKey("length"))
                {
                    radius = configMap["radius"];
                    length = configMap["length"];
                }
                else
                {
                    LOG_ERROR_S << "The config map has no key 'radius' and/or 'length'";
                    radius = 0.;
                    length = 0.;
                }
            }
            else
            {
                LOG_ERROR_S << "The config map has no key 'type' or wrong type value";
                radius = 0.;
                length = 0.;
            }
        }
        Cylinder(urdf::CylinderSharedPtr urdfGeometry) : Geometry(Geometry::CYLINDER)
        {
            if (urdfGeometry != nullptr)
            {
                radius = urdfGeometry->radius;
                length = urdfGeometry->length;
            }
            else
            {
                // TODO: show the warning
                radius = 0.;
                length = 0.;
            }
        }

        double radius;
        double length;

        // TODO: is extend not mars specific config map parameter?
        // can we use radius and length?
        configmaps::ConfigMap getConfigMap() const override
        {
            configmaps::ConfigMap configMap = Geometry::getConfigMap();
            configMap["type"] = toString(type);
            configMap["radius"] = radius;
            configMap["lenght"] = length;
            return configMap;
        }
    };

    struct Mesh : public Geometry
    {
        Mesh() : Mesh(std::string()) {}
        Mesh(std::string filename) : Mesh(filename, base::Vector3d(1., 1., 1.)) {}
        Mesh(std::string filename, base::Vector3d scale) : Geometry(Geometry::MESH),
                                                           filename(filename), scale(scale) {}
        Mesh(configmaps::ConfigMap &configMap) : Geometry(Geometry::MESH)
        {
            if (configMap.hasKey("type") && isType(configMap["type"].toString()))
            {
                if (configMap.hasKey("filename") && configMap.hasKey("scale") &&
                    configMap["scale"].hasKey("x") && configMap["scale"].hasKey("y") && configMap["scale"].hasKey("z"))
                {
                    filename = configMap["filename"].toString();
                    scale.x() = configMap["scale"]["x"];
                    scale.y() = configMap["scale"]["y"];
                    scale.z() = configMap["scale"]["z"];
                }
                else
                {
                    LOG_ERROR_S << "The config map has no key 'filename' and/or 'scale' or the key 'scale' has wrong structure";
                    filename = std::string();
                    scale = base::Vector3d::Zero();
                }
            }
            else
            {
                LOG_ERROR_S << "The config map has no key 'type' or wrong type value";
                filename = std::string();
                scale = base::Vector3d::Zero();
            }
        }
        Mesh(urdf::MeshSharedPtr urdfGeometry) : Geometry(Geometry::MESH)
        {
            if (urdfGeometry != nullptr)
            {
                filename = urdfGeometry->filename;
                scale.x() = urdfGeometry->scale.x;
                scale.y() = urdfGeometry->scale.y;
                scale.z() = urdfGeometry->scale.z;
            }
            else
            {
                // TODO: show the warning
                filename = std::string();
                scale = base::Vector3d::Zero();
            }
        }

        std::string filename;
        base::Vector3d scale;

        // TODO: is extend not mars specific config map parameter?
        // can we use radius and length?
        configmaps::ConfigMap getConfigMap() const override
        {
            configmaps::ConfigMap configMap = Geometry::getConfigMap();
            configMap["filename"] = filename;
            configMap["type"] = toString(type);
            configMap["scale"]["x"] = scale.x();
            configMap["scale"]["y"] = scale.y();
            configMap["scale"]["z"] = scale.z();
            return configMap;
        }
    };

    struct Plane : public Geometry
    {
        Plane() : Plane(base::Vector2d(0., 0.)) {}
        Plane(base::Vector2d size) : Geometry(Geometry::PLANE), size(size) {}
        Plane(configmaps::ConfigMap &configMap) : Geometry(Geometry::PLANE)
        {
            if (configMap.hasKey("type") && isType(configMap["type"].toString()))
            {
                if (configMap.hasKey("size") && configMap["size"].hasKey("x") && configMap["size"].hasKey("y"))
                {
                    size.x() = configMap["size"]["x"];
                    size.y() = configMap["size"]["y"];
                }
                else
                {
                    LOG_ERROR_S << "The config map has no or wrong structure of the key 'size'";
                    size = base::Vector2d::Zero();
                }
            }
            else
            {
                LOG_ERROR_S << "The config map has no key 'type' or wrong type value";
                size = base::Vector2d::Zero();
            }
        }

        base::Vector2d size;

        // TODO: is extend not mars specific config map parameter?
        // can we use radius and length?
        configmaps::ConfigMap getConfigMap() const override
        {
            configmaps::ConfigMap configMap = Geometry::getConfigMap();
            configMap["type"] = toString(type);
            configMap["size"]["x"] = size.x();
            configMap["size"]["y"] = size.y();

            return configMap;
        }
    };

    struct Sphere : public Geometry
    {
        Sphere() : Sphere(0.) {}
        Sphere(double radius) : Geometry(Geometry::SPHERE), radius(radius) {}
        Sphere(configmaps::ConfigMap &configMap) : Geometry(Geometry::SPHERE)
        {
            if (configMap.hasKey("type") && isType(configMap["type"].toString())) {
                if (configMap.hasKey("radius"))
                {
                    radius = configMap["radius"];
                }
                else
                {
                    LOG_ERROR_S << "The config map has no key 'radius'";
                    radius = 0.;
                }
            }else
            {
                LOG_ERROR_S << "The config map has no key 'type' or wrong type value";
                radius = 0.;
            }
        }
        Sphere(urdf::SphereSharedPtr urdfGeometry) : Geometry(Geometry::SPHERE)
        {
            if (urdfGeometry != nullptr)
                radius = urdfGeometry->radius;
            else
            {
                // TODO: show the warning
                radius = 0.;
            }
        }

        double radius;

        // TODO: is extend not mars specific config map parameter?
        // can we use radius?
        configmaps::ConfigMap getConfigMap() const override
        {
            configmaps::ConfigMap configMap = Geometry::getConfigMap();
            configMap["type"] = toString(type);
            configMap["radius"] = radius;
            return configMap;
        }
    };
}
#endif // GEOMETRY_HPP