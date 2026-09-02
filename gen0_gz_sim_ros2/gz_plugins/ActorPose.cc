#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/common/Profiler.hh>

#include <gz/sim/Actor.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>

#include "ActorPose.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

namespace
{
struct TrajectoryWaypoint
{
  double time{0.0};
  math::Pose3d pose;
};

bool EnvBool(const char *_name, bool _default)
{
  const char *value = std::getenv(_name);
  if (!value)
    return _default;

  std::string text(value);
  std::transform(text.begin(), text.end(), text.begin(),
      [](unsigned char c) { return std::tolower(c); });
  return text == "1" || text == "true" || text == "yes" || text == "on";
}

double EnvDouble(const char *_name, double _default)
{
  const char *value = std::getenv(_name);
  if (!value)
    return _default;

  try
  {
    return std::stod(value);
  }
  catch (...)
  {
    ignwarn << "Invalid " << _name << "=" << value
            << ", using " << _default << std::endl;
    return _default;
  }
}

std::string EnvString(const char *_name, const std::string &_default)
{
  const char *value = std::getenv(_name);
  return value ? std::string(value) : _default;
}

double NormalizeScriptTime(double _time, double _first, double _last, bool _loop)
{
  if (!_loop || _last <= _first)
    return _time;

  const double period = _last - _first;
  double normalized = std::fmod(_time - _first, period);
  if (normalized < 0.0)
    normalized += period;
  return normalized + _first;
}

math::Pose3d InterpolatePose(const std::vector<TrajectoryWaypoint> &_waypoints,
    double _time, bool _loop)
{
  if (_waypoints.empty())
    return math::Pose3d();

  const double first = _waypoints.front().time;
  const double last = _waypoints.back().time;
  const double t = NormalizeScriptTime(_time, first, last, _loop);

  if (t <= first)
    return _waypoints.front().pose;
  if (t >= last)
    return _waypoints.back().pose;

  for (size_t i = 0; i + 1 < _waypoints.size(); ++i)
  {
    const auto &a = _waypoints[i];
    const auto &b = _waypoints[i + 1];
    if (t < a.time || t > b.time)
      continue;

    const double span = b.time - a.time;
    const double ratio = span > 0.0 ? (t - a.time) / span : 1.0;
    const auto pos = a.pose.Pos() * (1.0 - ratio) + b.pose.Pos() * ratio;
    const auto rot = math::Quaterniond::Slerp(ratio, a.pose.Rot(), b.pose.Rot(), true);
    return math::Pose3d(pos, rot);
  }

  return _waypoints.back().pose;
}

double ActorVehicleClearance(const math::Pose3d &_actorPose,
    const math::Pose3d &_vehiclePose, double _vehicleLength,
    double _vehicleWidth, double _vehiclePadding, double _actorRadius)
{
  const double yaw = _vehiclePose.Rot().Yaw();
  const double cosYaw = std::cos(yaw);
  const double sinYaw = std::sin(yaw);
  const double relX = _actorPose.X() - _vehiclePose.X();
  const double relY = _actorPose.Y() - _vehiclePose.Y();
  const double localX = cosYaw * relX + sinYaw * relY;
  const double localY = -sinYaw * relX + cosYaw * relY;
  const double halfLength = _vehicleLength * 0.5 + _vehiclePadding;
  const double halfWidth = _vehicleWidth * 0.5 + _vehiclePadding;
  const double insideX = halfLength - std::abs(localX);
  const double insideY = halfWidth - std::abs(localY);

  if (insideX >= 0.0 && insideY >= 0.0)
    return -std::min(insideX, insideY) - _actorRadius;

  const double outsideX = std::max(-insideX, 0.0);
  const double outsideY = std::max(-insideY, 0.0);
  return std::hypot(outsideX, outsideY) - _actorRadius;
}

math::Pose3d ClampActorOutsideVehicle(const math::Pose3d &_actorPose,
    const math::Pose3d &_vehiclePose, double _vehicleLength,
    double _vehicleWidth, double _vehiclePadding, double _actorRadius,
    double _stopMargin)
{
  const double yaw = _vehiclePose.Rot().Yaw();
  const double cosYaw = std::cos(yaw);
  const double sinYaw = std::sin(yaw);
  const double relX = _actorPose.X() - _vehiclePose.X();
  const double relY = _actorPose.Y() - _vehiclePose.Y();
  double localX = cosYaw * relX + sinYaw * relY;
  double localY = -sinYaw * relX + cosYaw * relY;
  const double halfLength = _vehicleLength * 0.5 + _vehiclePadding;
  const double halfWidth = _vehicleWidth * 0.5 + _vehiclePadding;
  const double targetDistance = _actorRadius + std::max(0.0, _stopMargin);

  const double closestX = std::clamp(localX, -halfLength, halfLength);
  const double closestY = std::clamp(localY, -halfWidth, halfWidth);
  double awayX = localX - closestX;
  double awayY = localY - closestY;
  double awayNorm = std::hypot(awayX, awayY);

  if (awayNorm < 1e-6)
  {
    const double distX = halfLength - std::abs(localX);
    const double distY = halfWidth - std::abs(localY);
    if (distX < distY)
    {
      awayX = localX >= 0.0 ? 1.0 : -1.0;
      awayY = 0.0;
      localX = awayX * (halfLength + targetDistance);
    }
    else
    {
      awayX = 0.0;
      awayY = localY >= 0.0 ? 1.0 : -1.0;
      localY = awayY * (halfWidth + targetDistance);
    }
  }
  else
  {
    awayX /= awayNorm;
    awayY /= awayNorm;
    localX = closestX + awayX * targetDistance;
    localY = closestY + awayY * targetDistance;
  }

  math::Pose3d adjusted = _actorPose;
  adjusted.SetX(_vehiclePose.X() + cosYaw * localX - sinYaw * localY);
  adjusted.SetY(_vehiclePose.Y() + sinYaw * localX + cosYaw * localY);
  return adjusted;
}
}

class ignition::gazebo::systems::ActorPosePrivate
{
  /// \brief Entity for the actor.
  public: Entity actorEntity{kNullEntity};

  public: Entity vehicleEntity{kNullEntity};

  public: Entity collisionProxyEntity{kNullEntity};
  
  public: msgs::Pose poseMsg;

  public: transport::Node node;

  public: transport::Node::Publisher posePub;
  
  public: double updateFrequency = -1;

  public: std::chrono::steady_clock::duration updatePeriod{0};

  /// \brief Time of the last update.
  public: std::chrono::steady_clock::duration lastUpdate{0};

  public: bool softStop{false};

  public: std::string vehicleName{"gen0_model"};

  public: double vehicleLength{4.0};

  public: double vehicleWidth{2.0};

  public: double vehiclePadding{0.05};

  public: double actorRadius{0.45};

  public: double stopMargin{0.25};

  public: double releaseMargin{0.85};

  public: bool manualTrajectoryLoaded{false};

  public: bool manualTrajectoryWarned{false};

  public: bool blocked{false};

  public: math::Pose3d blockedPose;

  public: bool scriptLoop{true};

  public: double scriptDelayStart{0.0};

  public: std::vector<TrajectoryWaypoint> waypoints;

};

//////////////////////////////////////////////////
ActorPose::ActorPose() :
  System(), dataPtr(std::make_unique<ActorPosePrivate>())
{
}

//////////////////////////////////////////////////
ActorPose::~ActorPose() = default;

//////////////////////////////////////////////////
void ActorPose::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  (void)_sdf;
  this->dataPtr->actorEntity = _entity;
  this->dataPtr->softStop = EnvBool("GEN0_ACTOR_SOFT_STOP", false);
  this->dataPtr->vehicleName =
      EnvString("GEN0_ACTOR_SOFT_STOP_VEHICLE_NAME", "gen0_model");
  this->dataPtr->vehicleLength =
      std::max(0.1, EnvDouble("GEN0_ACTOR_SOFT_STOP_VEHICLE_LENGTH", 4.0));
  this->dataPtr->vehicleWidth =
      std::max(0.1, EnvDouble("GEN0_ACTOR_SOFT_STOP_VEHICLE_WIDTH", 2.0));
  this->dataPtr->vehiclePadding =
      std::max(0.0, EnvDouble("GEN0_ACTOR_SOFT_STOP_VEHICLE_PADDING", 0.05));
  this->dataPtr->actorRadius =
      std::max(0.05, EnvDouble("GEN0_ACTOR_SOFT_STOP_ACTOR_RADIUS", 0.45));
  this->dataPtr->stopMargin =
      std::max(0.0, EnvDouble("GEN0_ACTOR_SOFT_STOP_MARGIN", 0.25));
  this->dataPtr->releaseMargin =
      std::max(this->dataPtr->stopMargin,
          EnvDouble("GEN0_ACTOR_SOFT_STOP_RELEASE_MARGIN", 0.85));

  std::string poseTopic = scopedName(_entity, _ecm) + "/pose";

  poseTopic = transport::TopicUtils::AsValidTopic(poseTopic);

  this->dataPtr->posePub = this->dataPtr->node.Advertise<msgs::Pose>(poseTopic);

  if (this->dataPtr->updateFrequency > 0)
  {
    std::chrono::duration<double> period{1 / this->dataPtr->updateFrequency};
    this->dataPtr->updatePeriod =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
  }

  this->dataPtr->vehicleEntity = _ecm.EntityByComponents(
      components::Name(this->dataPtr->vehicleName), components::Model());

  const auto *actorName = _ecm.Component<components::Name>(_entity);
  if (actorName)
  {
    this->dataPtr->collisionProxyEntity = _ecm.EntityByComponents(
        components::Name(actorName->Data() + "_collision_proxy"),
        components::Model());
  }

  if (const auto *actorComponent =
      _ecm.Component<components::Actor>(this->dataPtr->actorEntity))
  {
    const auto &sdfActor = actorComponent->Data();
    this->dataPtr->scriptLoop = sdfActor.ScriptLoop();
    this->dataPtr->scriptDelayStart = sdfActor.ScriptDelayStart();

    for (uint64_t trajIndex = 0; trajIndex < sdfActor.TrajectoryCount();
         ++trajIndex)
    {
      const auto *trajectory = sdfActor.TrajectoryByIndex(trajIndex);
      if (!trajectory || trajectory->WaypointCount() == 0)
        continue;

      this->dataPtr->waypoints.clear();
      this->dataPtr->waypoints.reserve(trajectory->WaypointCount());
      for (uint64_t wpIndex = 0; wpIndex < trajectory->WaypointCount();
           ++wpIndex)
      {
        const auto *waypoint = trajectory->WaypointByIndex(wpIndex);
        if (!waypoint)
          continue;
        this->dataPtr->waypoints.push_back(
            {waypoint->Time(), waypoint->Pose()});
      }
      std::sort(this->dataPtr->waypoints.begin(),
          this->dataPtr->waypoints.end(),
          [](const auto &_a, const auto &_b) { return _a.time < _b.time; });
      this->dataPtr->manualTrajectoryLoaded =
          !this->dataPtr->waypoints.empty();
      break;
    }
  }

  if (this->dataPtr->softStop)
  {
    const auto *name = _ecm.Component<components::Name>(this->dataPtr->actorEntity);
    ignmsg << "Actor soft-stop enabled for "
           << (name ? name->Data() : std::string("actor"))
           << ": vehicle=" << this->dataPtr->vehicleName
           << ", vehicle_box=" << this->dataPtr->vehicleLength << "x"
           << this->dataPtr->vehicleWidth
           << ", actor_radius=" << this->dataPtr->actorRadius
           << ", stop_margin=" << this->dataPtr->stopMargin
           << ", release_margin=" << this->dataPtr->releaseMargin
           << ", trajectory_waypoints=" << this->dataPtr->waypoints.size()
           << std::endl;
  }

}

//////////////////////////////////////////////////

void ActorPose::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  IGN_PROFILE("ActorPose::PreUpdate");

  if (_info.paused)
    return;

  if (this->dataPtr->collisionProxyEntity == kNullEntity)
  {
    const auto *actorName = _ecm.Component<components::Name>(
        this->dataPtr->actorEntity);
    if (actorName)
    {
      this->dataPtr->collisionProxyEntity = _ecm.EntityByComponents(
          components::Name(actorName->Data() + "_collision_proxy"),
          components::Model());
    }
  }

  // Keep the hidden collision body aligned even when the optional soft-stop
  // behavior is disabled.
  if (!this->dataPtr->softStop)
  {
    if (this->dataPtr->collisionProxyEntity != kNullEntity)
    {
      const auto *actorWorldPose = _ecm.Component<components::WorldPose>(
          this->dataPtr->actorEntity);
      const auto *actorLocalPose = _ecm.Component<components::Pose>(
          this->dataPtr->actorEntity);
      if (actorWorldPose || actorLocalPose)
      {
        const math::Pose3d actorPose = actorWorldPose ?
            actorWorldPose->Data() : actorLocalPose->Data();
        _ecm.SetComponentData<components::Pose>(
            this->dataPtr->collisionProxyEntity, actorPose);
      }
    }
    return;
  }

  if (!this->dataPtr->manualTrajectoryLoaded)
  {
    if (!this->dataPtr->manualTrajectoryWarned)
    {
      const auto *name =
          _ecm.Component<components::Name>(this->dataPtr->actorEntity);
      ignwarn << "Actor soft-stop requested for "
              << (name ? name->Data() : std::string("actor"))
              << ", but no script trajectory was found; leaving Gazebo script "
              << "in control." << std::endl;
      this->dataPtr->manualTrajectoryWarned = true;
    }
    return;
  }

  if (this->dataPtr->vehicleEntity == kNullEntity)
  {
    this->dataPtr->vehicleEntity = _ecm.EntityByComponents(
        components::Name(this->dataPtr->vehicleName), components::Model());
    if (this->dataPtr->vehicleEntity == kNullEntity)
      return;
  }

  const auto *vehicleWorldPose =
      _ecm.Component<components::WorldPose>(this->dataPtr->vehicleEntity);
  const auto *vehicleLocalPose =
      _ecm.Component<components::Pose>(this->dataPtr->vehicleEntity);
  if (!vehicleWorldPose && !vehicleLocalPose)
    return;

  const math::Pose3d vehiclePose =
      vehicleWorldPose ? vehicleWorldPose->Data() : vehicleLocalPose->Data();

  const double dt = std::max(
      0.0, std::chrono::duration<double>(_info.dt).count());
  const double currentTime =
      std::chrono::duration<double>(_info.simTime).count();
  const double candidateTime = currentTime + dt;
  math::Pose3d currentPose = InterpolatePose(
      this->dataPtr->waypoints, currentTime - this->dataPtr->scriptDelayStart,
      this->dataPtr->scriptLoop);
  math::Pose3d candidatePose = InterpolatePose(
      this->dataPtr->waypoints,
      candidateTime - this->dataPtr->scriptDelayStart,
      this->dataPtr->scriptLoop);

  const double currentClearance = ActorVehicleClearance(
      currentPose, vehiclePose, this->dataPtr->vehicleLength,
      this->dataPtr->vehicleWidth, this->dataPtr->vehiclePadding,
      this->dataPtr->actorRadius);
  const double candidateClearance = ActorVehicleClearance(
      candidatePose, vehiclePose, this->dataPtr->vehicleLength,
      this->dataPtr->vehicleWidth, this->dataPtr->vehiclePadding,
      this->dataPtr->actorRadius);

  if (!this->dataPtr->blocked)
  {
    // Leave the actor under Gazebo's native script unless its next step would
    // enter the vehicle safety envelope.
    if (currentClearance > 0.0 &&
        candidateClearance > this->dataPtr->stopMargin)
      return;

    this->dataPtr->blockedPose = currentPose;
    if (currentClearance <= this->dataPtr->stopMargin)
    {
      this->dataPtr->blockedPose = ClampActorOutsideVehicle(
          currentPose, vehiclePose, this->dataPtr->vehicleLength,
          this->dataPtr->vehicleWidth, this->dataPtr->vehiclePadding,
          this->dataPtr->actorRadius, this->dataPtr->stopMargin);
    }
    this->dataPtr->blocked = true;
    const auto *name =
        _ecm.Component<components::Name>(this->dataPtr->actorEntity);
    ignmsg << "Actor soft-stop holding "
           << (name ? name->Data() : std::string("actor"))
           << ", clearance=" << currentClearance << std::endl;
  }
  else
  {
    const double blockedClearance = ActorVehicleClearance(
        this->dataPtr->blockedPose, vehiclePose, this->dataPtr->vehicleLength,
        this->dataPtr->vehicleWidth, this->dataPtr->vehiclePadding,
        this->dataPtr->actorRadius);
    if (blockedClearance > this->dataPtr->releaseMargin &&
        candidateClearance > this->dataPtr->stopMargin)
    {
      // Remove the manual override so Gazebo can resume its SDF trajectory.
      _ecm.RemoveComponent<components::TrajectoryPose>(
          this->dataPtr->actorEntity);
      this->dataPtr->blocked = false;
      const auto *name =
          _ecm.Component<components::Name>(this->dataPtr->actorEntity);
      ignmsg << "Actor soft-stop released "
             << (name ? name->Data() : std::string("actor"))
             << ", clearance=" << blockedClearance << std::endl;
      return;
    }
  }

  if (this->dataPtr->blocked)
  {
    Actor actor(this->dataPtr->actorEntity);
    actor.SetTrajectoryPose(_ecm, this->dataPtr->blockedPose);
  }

  if (this->dataPtr->collisionProxyEntity != kNullEntity)
  {
    math::Pose3d proxyPose = currentPose;
    if (this->dataPtr->blocked)
      proxyPose = this->dataPtr->blockedPose;
    _ecm.SetComponentData<components::Pose>(
        this->dataPtr->collisionProxyEntity, proxyPose);
  }
}

//////////////////////////////////////////////////

void ActorPose::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
    IGN_PROFILE("ActorPose::PostUpdate");

    if (_info.dt < std::chrono::steady_clock::duration::zero())
    {
        ignwarn << "Detected jump back in time ["
            << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
            << "s]. System may not work properly." << std::endl;
    }

    if (_info.paused)
    return;

    bool publish = true;

    auto diff = _info.simTime - this->dataPtr->lastUpdate;

    if ((diff > std::chrono::steady_clock::duration::zero()) &&
      (diff < this->dataPtr->updatePeriod))
    {
        publish = false;
    }
    if (!publish)
        return;
    
    const auto *actorPose =
        _ecm.Component<components::WorldPose>(this->dataPtr->actorEntity);
    const auto *localPose =
        _ecm.Component<components::Pose>(this->dataPtr->actorEntity);
    const auto *name =
        _ecm.Component<components::Name>(this->dataPtr->actorEntity);

    if ((!actorPose && !localPose) || !name)
    {
        return;
    }

    msgs::Pose *msg = nullptr;
    this->dataPtr->poseMsg.Clear();
    msg = &this->dataPtr->poseMsg;
    
    auto timeStamp = convert<msgs::Time>(_info.simTime);
    auto header = msg->mutable_header();
    header->mutable_stamp()->CopyFrom(timeStamp);
    const math::Pose3d transform = actorPose ? actorPose->Data() : localPose->Data();
    msg->set_name(name->Data());
    msgs::Set(msg, transform);


    this->dataPtr->posePub.Publish(this->dataPtr->poseMsg);

    // this->dataPtr->PublishPoses(this->dataPtr->poses, convert<msgs::Time>(_info.simTime), this->dataPtr->posePub);

    this->dataPtr->lastUpdate = _info.simTime;
}

IGNITION_ADD_PLUGIN(ActorPose, System,
  ActorPose::ISystemConfigure,
  ActorPose::ISystemPreUpdate,
  ActorPose::ISystemPostUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(ActorPose, "ignition::gazebo::systems::ActorPose")
