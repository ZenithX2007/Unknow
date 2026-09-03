#ifndef IGNITION_GAZEBO_SYSTEMS_TRAFFICPOSE_HH_
#define IGNITION_GAZEBO_SYSTEMS_TRAFFICPOSE_HH_

#include <memory>

#include <gz/sim/config.hh>
#include <gz/sim/System.hh>
#include <sdf/Element.hh>

namespace ignition
{
namespace gazebo
{
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  class TrafficPosePrivate;

  class TrafficPose:
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate
  {
    public: explicit TrafficPose();

    public: ~TrafficPose() override;

    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) final;

    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) final;

    private: std::unique_ptr<TrafficPosePrivate> dataPtr;
  };
}
}
}
}

#endif
