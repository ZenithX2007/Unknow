#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>

#include <sdf/Element.hh>

#include "TrafficPose.hh"

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

struct ClearanceInfo
{
  bool found{false};
  double clearance{std::numeric_limits<double>::infinity()};
  math::Pose3d obstaclePose;
  std::string obstacleName;
  double obstacleRadius{0.0};
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

bool StartsWith(const std::string &_text, const std::string &_prefix)
{
  return _text.size() >= _prefix.size() &&
      std::equal(_prefix.begin(), _prefix.end(), _text.begin());
}

double ObstacleRadiusFromName(const std::string &_name)
{
  if (_name == "gen0_model")
    return 1.6;
  if (StartsWith(_name, "pedestrian_"))
    return 0.45;
  if (StartsWith(_name, "car_"))
    return 1.6;
  return 0.0;
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

double VehicleObstacleClearance(const math::Pose3d &_vehiclePose,
    const math::Pose3d &_obstaclePose, double _vehicleLength,
    double _vehicleWidth, double _vehiclePadding, double _obstacleRadius)
{
  const double yaw = _vehiclePose.Rot().Yaw();
  const double cosYaw = std::cos(yaw);
  const double sinYaw = std::sin(yaw);
  const double relX = _obstaclePose.X() - _vehiclePose.X();
  const double relY = _obstaclePose.Y() - _vehiclePose.Y();
  const double localX = cosYaw * relX + sinYaw * relY;
  const double localY = -sinYaw * relX + cosYaw * relY;
  const double halfLength = _vehicleLength * 0.5 + _vehiclePadding;
  const double halfWidth = _vehicleWidth * 0.5 + _vehiclePadding;
  const double insideX = halfLength - std::abs(localX);
  const double insideY = halfWidth - std::abs(localY);

  if (insideX >= 0.0 && insideY >= 0.0)
    return -std::min(insideX, insideY) - _obstacleRadius;

  const double outsideX = std::max(-insideX, 0.0);
  const double outsideY = std::max(-insideY, 0.0);
  return std::hypot(outsideX, outsideY) - _obstacleRadius;
}

template <typename T>
T GetParamOrDefault(const std::shared_ptr<const sdf::Element> &_sdf,
    const std::string &_name, const T &_default)
{
  if (_sdf && _sdf->HasElement(_name))
  {
    T value = _default;
    if (_sdf->Get<T>(_name, value, _default))
      return value;
  }
  return _default;
}
}

class ignition::gazebo::systems::TrafficPosePrivate
{
  public: Entity modelEntity{kNullEntity};

  public: bool manualTrajectoryLoaded{false};

  public: bool manualTrajectoryWarned{false};

  public: bool manualTimeInitialized{false};

  public: double manualTime{0.0};

  public: bool blocked{false};

  public: bool scriptLoop{true};

  public: double scriptDelayStart{0.0};

  public: double vehicleLength{4.4};

  public: double vehicleWidth{2.2};

  public: double vehiclePadding{0.20};

  public: double stopMargin{0.25};

  public: double releaseMargin{0.85};

  public: std::vector<TrajectoryWaypoint> waypoints;
};

//////////////////////////////////////////////////
TrafficPose::TrafficPose() :
  System(), dataPtr(std::make_unique<TrafficPosePrivate>())
{
}

//////////////////////////////////////////////////
TrafficPose::~TrafficPose() = default;

//////////////////////////////////////////////////
void TrafficPose::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->modelEntity = _entity;
  const auto *name = _ecm.Component<components::Name>(this->dataPtr->modelEntity);
  const std::string modelName = name ? name->Data() : std::string("traffic_car");

  this->dataPtr->vehicleLength = std::max(
      0.1, GetParamOrDefault<double>(_sdf, "vehicle_length",
          EnvDouble("GEN0_TRAFFIC_CAR_LENGTH", this->dataPtr->vehicleLength)));
  this->dataPtr->vehicleWidth = std::max(
      0.1, GetParamOrDefault<double>(_sdf, "vehicle_width",
          EnvDouble("GEN0_TRAFFIC_CAR_WIDTH", this->dataPtr->vehicleWidth)));
  this->dataPtr->vehiclePadding = std::max(
      0.0, GetParamOrDefault<double>(_sdf, "vehicle_padding",
          EnvDouble("GEN0_TRAFFIC_CAR_PADDING", this->dataPtr->vehiclePadding)));
  this->dataPtr->stopMargin = std::max(
      0.0, GetParamOrDefault<double>(_sdf, "stop_margin",
          EnvDouble("GEN0_TRAFFIC_CAR_STOP_MARGIN", this->dataPtr->stopMargin)));
  this->dataPtr->releaseMargin = std::max(
      this->dataPtr->stopMargin,
      GetParamOrDefault<double>(_sdf, "release_margin",
          EnvDouble("GEN0_TRAFFIC_CAR_RELEASE_MARGIN",
              this->dataPtr->releaseMargin)));

  if (_sdf && _sdf->HasElement("trajectory"))
  {
    const auto trajectory = _sdf->FindElement("trajectory");
    if (trajectory)
    {
      this->dataPtr->scriptLoop = GetParamOrDefault<bool>(
          trajectory, "loop",
          EnvBool("GEN0_TRAFFIC_CAR_LOOP", true));
      this->dataPtr->scriptDelayStart = std::max(
          0.0, GetParamOrDefault<double>(trajectory, "delay_start",
              EnvDouble("GEN0_TRAFFIC_CAR_DELAY_START", 0.0)));

      for (auto waypoint = trajectory->GetFirstElement();
           waypoint; waypoint = waypoint->GetNextElement())
      {
        if (waypoint->GetName() != "waypoint")
          continue;

        double time = 0.0;
        std::string poseText;
        if (!waypoint->Get<double>("time", time, 0.0))
          continue;
        if (!waypoint->Get<std::string>("pose", poseText, ""))
          continue;

        std::vector<double> poseValues;
        poseValues.reserve(6);
        try
        {
          size_t start = 0;
          while (start < poseText.size())
          {
            while (start < poseText.size() &&
                std::isspace(static_cast<unsigned char>(poseText[start])))
              ++start;
            if (start >= poseText.size())
              break;

            size_t end = start;
            while (end < poseText.size() &&
                !std::isspace(static_cast<unsigned char>(poseText[end])))
              ++end;

            poseValues.push_back(std::stod(poseText.substr(start, end - start)));
            start = end;
          }
        }
        catch (...)
        {
          poseValues.clear();
        }

        if (poseValues.size() < 2)
          continue;
        while (poseValues.size() < 6)
          poseValues.push_back(0.0);

        this->dataPtr->waypoints.push_back({
            time,
            math::Pose3d(
              poseValues[0], poseValues[1], poseValues[2],
              poseValues[3], poseValues[4], poseValues[5])
        });
      }
    }
  }

  std::sort(this->dataPtr->waypoints.begin(), this->dataPtr->waypoints.end(),
      [](const auto &_a, const auto &_b) { return _a.time < _b.time; });
  this->dataPtr->manualTrajectoryLoaded =
      !this->dataPtr->waypoints.empty();

  if (this->dataPtr->manualTrajectoryLoaded)
  {
    ignmsg << "Traffic vehicle motion enabled for " << modelName
           << ": waypoints=" << this->dataPtr->waypoints.size()
           << ", vehicle_box=" << this->dataPtr->vehicleLength << "x"
           << this->dataPtr->vehicleWidth
           << ", padding=" << this->dataPtr->vehiclePadding
           << ", stop_margin=" << this->dataPtr->stopMargin
           << ", release_margin=" << this->dataPtr->releaseMargin
           << ", loop=" << this->dataPtr->scriptLoop
           << ", delay_start=" << this->dataPtr->scriptDelayStart
           << std::endl;
  }
  else if (!this->dataPtr->manualTrajectoryWarned)
  {
    ignwarn << "Traffic vehicle plugin attached to " << modelName
            << " but no trajectory was provided; the model will stay put."
            << std::endl;
    this->dataPtr->manualTrajectoryWarned = true;
  }
}

//////////////////////////////////////////////////
void TrafficPose::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  IGN_PROFILE("TrafficPose::PreUpdate");

  if (_info.paused || !this->dataPtr->manualTrajectoryLoaded)
    return;

  const auto *worldPose =
      _ecm.Component<components::WorldPose>(this->dataPtr->modelEntity);
  const auto *localPose =
      _ecm.Component<components::Pose>(this->dataPtr->modelEntity);
  if (!worldPose && !localPose)
    return;

  if (!this->dataPtr->manualTimeInitialized)
  {
    this->dataPtr->manualTime =
        std::chrono::duration<double>(_info.simTime).count();
    this->dataPtr->manualTimeInitialized = true;
  }

  const double dt = std::max(
      0.0, std::chrono::duration<double>(_info.dt).count());
  const double currentTime = this->dataPtr->manualTime;
  const double candidateTime = currentTime + dt;
  const math::Pose3d currentTrajectoryPose = InterpolatePose(
      this->dataPtr->waypoints, currentTime - this->dataPtr->scriptDelayStart,
      this->dataPtr->scriptLoop);
  const math::Pose3d candidateTrajectoryPose = InterpolatePose(
      this->dataPtr->waypoints,
      candidateTime - this->dataPtr->scriptDelayStart,
      this->dataPtr->scriptLoop);

  ClearanceInfo currentClearance;
  ClearanceInfo candidateClearance;

  _ecm.Each<components::Name, components::WorldPose>(
      [&](const Entity _entity, const components::Name *_name,
          const components::WorldPose *_pose) -> bool
      {
        if (_entity == this->dataPtr->modelEntity)
          return true;

        const std::string obstacleName = _name->Data();
        const double obstacleRadius = ObstacleRadiusFromName(obstacleName);
        if (obstacleRadius <= 0.0)
          return true;

        const math::Pose3d obstaclePose = _pose->Data();
        const double current = VehicleObstacleClearance(
            currentTrajectoryPose, obstaclePose,
            this->dataPtr->vehicleLength, this->dataPtr->vehicleWidth,
            this->dataPtr->vehiclePadding, obstacleRadius);
        const double candidate = VehicleObstacleClearance(
            candidateTrajectoryPose, obstaclePose,
            this->dataPtr->vehicleLength, this->dataPtr->vehicleWidth,
            this->dataPtr->vehiclePadding, obstacleRadius);

        if (!currentClearance.found || current < currentClearance.clearance)
        {
          currentClearance.found = true;
          currentClearance.clearance = current;
          currentClearance.obstaclePose = obstaclePose;
          currentClearance.obstacleName = obstacleName;
          currentClearance.obstacleRadius = obstacleRadius;
        }
        if (!candidateClearance.found ||
            candidate < candidateClearance.clearance)
        {
          candidateClearance.found = true;
          candidateClearance.clearance = candidate;
          candidateClearance.obstaclePose = obstaclePose;
          candidateClearance.obstacleName = obstacleName;
          candidateClearance.obstacleRadius = obstacleRadius;
        }
        return true;
      });

  bool blockThisStep = false;
  if (this->dataPtr->blocked)
  {
    if (currentClearance.found &&
        currentClearance.clearance <= this->dataPtr->releaseMargin)
    {
      blockThisStep = true;
    }
    else
    {
      blockThisStep = candidateClearance.found &&
          candidateClearance.clearance <= this->dataPtr->stopMargin;
    }
  }
  else
  {
    blockThisStep = (currentClearance.found &&
        currentClearance.clearance <= 0.0) ||
        (candidateClearance.found &&
        candidateClearance.clearance <= this->dataPtr->stopMargin);
  }

  math::Pose3d outputPose = candidateTrajectoryPose;
  const ClearanceInfo *activeClearance = &candidateClearance;
  if (blockThisStep)
  {
    outputPose = currentTrajectoryPose;
    activeClearance = &currentClearance;
  }
  else
  {
    this->dataPtr->manualTime = candidateTime;
  }

  if (blockThisStep != this->dataPtr->blocked)
  {
    const auto *name =
        _ecm.Component<components::Name>(this->dataPtr->modelEntity);
    ignmsg << "Traffic vehicle "
           << (blockThisStep ? "holding " : "released ")
           << (name ? name->Data() : std::string("car"))
           << ", obstacle="
           << (activeClearance->found ? activeClearance->obstacleName
                                      : std::string("none"))
           << ", clearance="
           << (activeClearance->found ? activeClearance->clearance : 0.0)
           << std::endl;
    this->dataPtr->blocked = blockThisStep;
  }

  Model model(this->dataPtr->modelEntity);
  model.SetWorldPoseCmd(_ecm, outputPose);
}

IGNITION_ADD_PLUGIN(TrafficPose, System,
  TrafficPose::ISystemConfigure,
  TrafficPose::ISystemPreUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(TrafficPose, "ignition::gazebo::systems::TrafficPose")
