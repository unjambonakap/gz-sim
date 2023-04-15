/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gz/math/Vector3.hh>
#include <pybind11/pybind11.h>

#include "EntityComponentManager.hh"
#include "EventManager.hh"
#include "Server.hh"
#include "ServerConfig.hh"
#include "TestFixture.hh"
#include "UpdateInfo.hh"
#include "Util.hh"
#include "World.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/AngularVelocityCmd.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/LinearVelocityCmd.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/PoseCmd.hh"
#include "gz/sim/components/Gravity.hh"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <iostream>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>

namespace py = pybind11;
#include <pybind11/pybind11.h>

#include <gz/msgs/wrench.pb.h>
#include "pybind11_protobuf/native_proto_caster.h"


namespace gz {
namespace sim {
namespace python {
template <typename T> void define_component(pybind11::object module, const char *name) {

  pybind11::class_<T>(module, name)
    .def_static(
      "Get", [](EntityComponentManager &ecm, Entity id) { return ecm.Component<T>(id); },
      py::return_value_policy::reference)
    .def_static(
      "Remove", [](EntityComponentManager &ecm, Entity id) { return ecm.RemoveComponent<T>(id); },
      py::return_value_policy::reference)

    .def_static(
      "GetOrCreate",
      [](EntityComponentManager &ecm, Entity id) {
        auto comp = ecm.Component<T>(id);
        if (!comp) {
          ecm.CreateComponent(id, T(typename T::Type()));
        }
        auto res = ecm.Component<T>(id);
        return res;
      },
      py::return_value_policy::reference)

    .def("data", static_cast<typename T::Type &(T::*)()>(&T::Data),
         py::return_value_policy::reference)
    .def("set_data", [](T &self, const typename T::Type &data) {
      self.SetData(data, [](const typename T::Type &a, const typename T::Type &b) {
        (void)a;
        (void)b;
        return false;
      });
    });
}

void defineSimModel(pybind11::object module) {
  pybind11::class_<gz::sim::Model>(module, "Model")
    .def(pybind11::init<gz::sim::Entity>())
    .def("link", &gz::sim::Model::CanonicalLink, "Get canonical link")
    .def("entity", &gz::sim::Model::Entity, "")
    .def("joint_by_name", &gz::sim::Model::JointByName, "Get canonical link")
    .def("link_by_name", &gz::sim::Model::LinkByName, "Get canonical link")
    .def("joints", &gz::sim::Model::Joints, "Get canonical link")
    .def("links", &gz::sim::Model::Links, "Get canonical link");

  pybind11::class_<gz::sim::Link>(module, "Link")
    .def(pybind11::init<gz::sim::Entity>())
    .def("entity", &gz::sim::Link::Entity, "")
    .def("world_pose", &gz::sim::Link::WorldPose, "Get world inertial pose")
    .def("world_inertial_pose", &gz::sim::Link::WorldInertialPose, "Get world inertial pose")
    .def("world_linear_velocity",
         static_cast<std::optional<math::Vector3d> (Link::*)(const EntityComponentManager &) const>(
           &gz::sim::Link::WorldLinearVelocity),
         "Get world inertial pose")
    .def("world_angular_velocity", &gz::sim::Link::WorldAngularVelocity, "Get world inertial pose")
    .def("set_linear_velocity", &gz::sim::Link::SetLinearVelocity, "Get world inertial pose")
    .def("set_angular_velocity", &gz::sim::Link::SetAngularVelocity, "Get world inertial pose")
    .def("world_linear_acceleration", &gz::sim::Link::WorldLinearAcceleration,
         "Get world inertial pose")
    .def("world_angular_acceleration", &gz::sim::Link::WorldAngularAcceleration,
         "Get world inertial pose");

  pybind11::class_<gz::sim::components::BaseComponent>(module, "BaseComponent")
    .def("type_id", &gz::sim::components::BaseComponent::TypeId, "Get world inertial pose");

  pybind11::class_<gz::sim::components::JointAxis, gz::sim::components::BaseComponent>(module,
                                                                                       "JointAxis")
    .def(
      "xyz", [](gz::sim::components::JointAxis *obj) { return obj->Data().Xyz(); },
      "Get world inertial pose");

  define_component<components::WorldPose>(module, "WorldPose");
  define_component<components::Pose>(module, "Pose");
  define_component<components::Inertial>(module, "Inertial");
  define_component<components::LinearVelocity>(module, "LinearVelocity");
  define_component<components::AngularVelocity>(module, "AngularVelocity");

  define_component<components::LinearVelocityCmd>(module, "LinearVelocityCmd");
  define_component<components::AngularVelocityCmd>(module, "AngularVelocityCmd");
  define_component<components::WorldPoseCmd>(module, "WorldPoseCmd");
  define_component<components::WorldLinearVelocityCmd>(module, "WorldLinearVelocityCmd");
  define_component<components::WorldAngularVelocityCmd>(module, "WorldAngularVelocityCmd");
  define_component<components::InertialCmd>(module, "InertialCmd");
  define_component<components::GravityCmd>(module, "GravityCmd");
  define_component<components::Gravity>(module, "Gravity");
  define_component<components::ExternalWorldWrenchCmd>(module, "ExternalWorldWrenchCmd");

}

} // namespace python
} // namespace sim
} // namespace gz

PYBIND11_MODULE(BINDINGS_MODULE_NAME, m) {
  m.doc() = "Gazebo Sim Python Library.";

  pybind11_protobuf::ImportNativeProtoCasters();

  gz::sim::python::defineSimEntityComponentManager(m);
  gz::sim::python::defineSimEventManager(m);
  gz::sim::python::defineSimServer(m);
  gz::sim::python::defineSimServerConfig(m);
  gz::sim::python::defineSimTestFixture(m);
  gz::sim::python::defineSimUpdateInfo(m);
  gz::sim::python::defineSimWorld(m);
  gz::sim::python::defineSimUtil(m);
  gz::sim::python::defineSimModel(m);
}
