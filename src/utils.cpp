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

#include "utils.hpp"

namespace smurf
{
    namespace utils
    {

        base::Pose convertPose(const urdf::Pose &pose)
        {
            base::Pose poseBase;
            poseBase.position = base::Position(pose.position.x,
                                               pose.position.y,
                                               pose.position.z);

            poseBase.orientation = base::Orientation(pose.rotation.w,
                                                     pose.rotation.x,
                                                     pose.rotation.y,
                                                     pose.rotation.z);
            return poseBase;
        }

        smurf::Geometry* createGeometry(configmaps::ConfigMap &configMap)
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

        smurf::Geometry* createGeometry(urdf::GeometrySharedPtr urdfGeometry)
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