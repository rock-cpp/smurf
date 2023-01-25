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

// TODO: replace origname in config map by geometrytype

namespace smurf
{
    class Geometry
    {
    public:
        // TODO: do we need TERRAIN and HEIGHTMAP type?
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

        Geometry() : type(UNKNOWN_TYPE) {}
        Geometry(GeometryType type) : type(type) {}
        virtual ~Geometry(void) {}

        GeometryType getType() const { return type; }

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

    private:
        GeometryType type;
    };

    class Box : public Geometry
    {
    public:
        Box() : Box(base::Vector3d::Zero()) {}
        Box(base::Vector3d size) : Geometry(Geometry::BOX), size(size) {}
        Box(configmaps::ConfigMap &configMap) : Geometry(Geometry::BOX)
        {
            if (configMap["origname"].toString() == "box")
            {
                size.x() = configMap["extend"]["x"];
                size.y() = configMap["extend"]["y"];
                size.z() = configMap["extend"]["z"];
            }
            else
            {
                size = base::Vector3d::Zero();
                // TODO: show the warning
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

        base::Vector3d getSize() const { return size; }

        configmaps::ConfigMap getConfigMap() const override
        {
            configmaps::ConfigMap configMap = Geometry::getConfigMap();
            configMap["origname"] = "box";
            configMap["extend"]["x"] = size.x();
            configMap["extend"]["y"] = size.y();
            configMap["extend"]["z"] = size.z();
            return configMap;
        }

    private:
        base::Vector3d size;
    };

    class Capsule : public Geometry
    {
    public:
        Capsule() : Capsule(0., 0.) {}
        Capsule(double radius, double length) : Geometry(Geometry::CAPSULE), radius(radius), length(length) {}
        Capsule(configmaps::ConfigMap &configMap) : Geometry(Geometry::CAPSULE)
        {
            if (configMap["origname"] == "capsule")
            {
                radius = configMap["extend"]["x"];
                length = configMap["extend"]["y"];
            }
            else
            {
                radius = 0.;
                length = 0.;
                // TODO: show the warning
            }
        }

        double getRadius() const { return radius; }
        double getLength() const { return length; }

        // TODO: is extend not mars specific config map parameter?
        // can we use radius and length?
        configmaps::ConfigMap getConfigMap() const override
        {
            configmaps::ConfigMap configMap = Geometry::getConfigMap();
            configMap["origname"] = "capsule";
            configMap["extend"]["x"] = radius;
            configMap["extend"]["y"] = length;
            return configMap;
        }

    private:
        double radius;
        double length;
    };

    class Cylinder : public Geometry
    {
    public:
        Cylinder() : Cylinder(0., 0.) {}
        Cylinder(double radius, double length) : Geometry(Geometry::CYLINDER), radius(radius), length(length) {}
        Cylinder(configmaps::ConfigMap &configMap) : Geometry(Geometry::CYLINDER)
        {
            if (configMap["origname"] == "capsule")
            {
                radius = configMap["extend"]["x"];
                length = configMap["extend"]["y"];
            }
            else
            {
                radius = 0.;
                length = 0.;
                // TODO: show the warning
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

        double getRadius() const { return radius; }
        double getLength() const { return length; }

        // TODO: is extend not mars specific config map parameter?
        // can we use radius and length?
        configmaps::ConfigMap getConfigMap() const override
        {
            configmaps::ConfigMap configMap = Geometry::getConfigMap();
            configMap["origname"] = "capsule";
            configMap["extend"]["x"] = radius;
            configMap["extend"]["y"] = length;
            return configMap;
        }

    private:
        double radius;
        double length;
    };

    class Mesh : public Geometry
    {
    public:
        Mesh() : Mesh(std::string()) {}
        Mesh(std::string filename) : Mesh(filename, base::Vector3d(1., 1., 1.)) {}
        Mesh(std::string filename, base::Vector3d scale) : Geometry(Geometry::MESH),
                                                           filename(filename), scale(scale) {}
        Mesh(configmaps::ConfigMap &configMap) : Geometry(Geometry::MESH)
        {
            if (configMap["origname"] == "mesh")
            {
                filename = configMap["filename"].toString();
                scale.x() = configMap["visualscale"]["x"];
                scale.y() = configMap["visualscale"]["y"];
                scale.z() = configMap["visualscale"]["z"];
            }
            else
            {
                filename = std::string();
                scale = base::Vector3d::Zero();
                // TODO: show the warning
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

        std::string getFilename() const { return filename; }
        void setFilename(std::string filename) { this->filename = filename; }
        base::Vector3d getScale() const { return scale; }

        // TODO: is extend not mars specific config map parameter?
        // can we use radius and length?
        configmaps::ConfigMap getConfigMap() const override
        {
            configmaps::ConfigMap configMap = Geometry::getConfigMap();
            configMap["filename"] = filename;
            configMap["origname"] = ""; // TODO: this is defined by mars, probably we dont need to set it
            configMap["visualscale"]["x"] = scale.x();
            configMap["visualscale"]["y"] = scale.y();
            configMap["visualscale"]["z"] = scale.z();
            return configMap;
        }

    private:
        std::string filename;
        base::Vector3d scale;
    };

    class Plane : public Geometry
    {
    public:
        Plane() : Plane(base::Vector2d(0., 0.)) {}
        Plane(base::Vector2d size) : Geometry(Geometry::PLANE), size(size) {}
        Plane(configmaps::ConfigMap &configMap) : Geometry(Geometry::PLANE)
        {
            if (configMap["origname"] == "plane")
            {
                size.x() = configMap["extend"]["x"];
                size.y() = configMap["extend"]["y"];
            }
            else
            {
                // TODO: show the warning
                size = base::Vector2d::Zero();
            }
        }

        base::Vector2d getSize() const { return size; }

        // TODO: is extend not mars specific config map parameter?
        // can we use radius and length?
        configmaps::ConfigMap getConfigMap() const override
        {
            configmaps::ConfigMap configMap = Geometry::getConfigMap();
            configMap["origname"] = "plane";
            configMap["extend"]["x"] = size.x();
            configMap["extend"]["y"] = size.y();

            return configMap;
        }

    private:
        base::Vector2d size;
    };

    class Sphere : public Geometry
    {
    public:
        Sphere() : Sphere(0.) {}
        Sphere(double radius) : Geometry(Geometry::SPHERE), radius(radius) {}
        Sphere(configmaps::ConfigMap &configMap) : Geometry(Geometry::SPHERE)
        {
            if (configMap["origname"] == "sphere")
                radius = configMap["extend"]["x"];
            else
            {
                // TODO: show the warning
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

        double getRadius() const { return radius; }

        // TODO: is extend not mars specific config map parameter?
        // can we use radius?
        configmaps::ConfigMap getConfigMap() const override
        {
            configmaps::ConfigMap configMap = Geometry::getConfigMap();
            configMap["origname"] = "sphere";
            configMap["extend"]["x"] = radius;
            return configMap;
        }

    private:
        double radius;
    };

    namespace utils
    {
        static smurf::Geometry* createGeometry(configmaps::ConfigMap &configMap)
        {
            std::string origname = configMap["origname"].toString();
            if (origname == "box")
                return new smurf::Box(configMap);
            else if (origname == "capsule")
                return new smurf::Capsule(configMap);
            else if (origname == "cylinder")
                return new smurf::Cylinder(configMap);
            else if (origname == "mesh")
                return new smurf::Mesh(configMap);
            else if (origname == "plane")
                return new smurf::Plane(configMap);
            else if (origname == "sphere")
                return new smurf::Sphere(configMap);
            else
                return new smurf::Geometry();
        }

        static smurf::Geometry* createGeometry(urdf::GeometrySharedPtr urdfGeometry)
        {
            switch (urdfGeometry->type)
            {
            case urdf::Geometry::BOX:
            {
                urdf::BoxSharedPtr box = urdf::dynamic_pointer_cast<urdf::Box>(urdfGeometry);
                return new smurf::Box(box);
            }
            case urdf::Geometry::CYLINDER:
            {
                urdf::CylinderSharedPtr cylinder = urdf::dynamic_pointer_cast<urdf::Cylinder>(urdfGeometry);
                return new smurf::Cylinder(cylinder);
            }
            case urdf::Geometry::MESH:
            {
                urdf::MeshSharedPtr mesh = urdf::dynamic_pointer_cast<urdf::Mesh>(urdfGeometry);
                // TODO: check filename is alreade absolute
                return new smurf::Mesh(mesh);
            }
            case urdf::Geometry::SPHERE:
            {
                urdf::SphereSharedPtr shpere = urdf::dynamic_pointer_cast<urdf::Sphere>(urdfGeometry);
                // TODO: do we need assert here? like
                return new smurf::Sphere(shpere);
            }
            default:
            {
                return new smurf::Geometry();
            }
            }
        }
    }
}
#endif // GEOMETRY_HPP