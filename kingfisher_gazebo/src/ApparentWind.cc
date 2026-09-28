// Copyright 2025 Luis Batista
//
// Licensed under the BSD License; see the package.xml for details.

#include "ApparentWind.hh"

#include <chrono>
#include <cmath>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/float.pb.h>
#include <gz/msgs/header.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/wrench.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Model.hh>
#include <gz/transport/Node.hh>
#include <sdf/Frame.hh>
#include <sdf/Model.hh>

#include "WindModel.hh"

using Duration = std::chrono::steady_clock::duration;

namespace kingfisher_gazebo
{
/// \brief True on the first call, then once per period of sim time.
struct Throttle
{
  Duration period{};
  Duration last{};
  bool started{false};

  bool Ready(const Duration &_now)
  {
    if (this->started && _now - this->last < this->period)
      return false;
    this->started = true;
    this->last = _now;
    return true;
  }
};

class ApparentWindPrivate
{
  public: void OnSpeed(const gz::msgs::Float &_msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->windSpeed = _msg.data();
  }

  public: void OnDirection(const gz::msgs::Float &_msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->windDirection = _msg.data();
  }

  public: bool ConfigureSail(const sdf::ElementConstPtr &_sdf,
                             const gz::sim::Model &_model,
                             gz::sim::EntityComponentManager &_ecm);

  public: bool ConfigureAnemometer(const sdf::ElementConstPtr &_sdf,
                                   const gz::sim::Entity &_model,
                                   gz::sim::EntityComponentManager &_ecm);

  public: gz::transport::Node node;

  /// \brief True wind from USVWind: m/s, and degrees counter-clockwise from
  /// east, where it goes. Zero until the first message.
  public: std::mutex mutex;
  public: double windSpeed{0.0};
  public: double windDirection{0.0};

  public: bool sailEnabled{false};
  public: gz::sim::Link sail;
  public: gz::math::Vector3d ce;
  public: double area{0.0};
  public: double rho{1.225};
  public: SailTable table;
  public: gz::transport::Node::Publisher wrenchPub;
  public: std::string sailFrameId;
  public: Throttle wrenchThrottle;

  public: bool anemometerEnabled{false};
  public: gz::sim::Link anemometerBody;
  /// \brief Anemometer frame in its body link's frame
  public: gz::math::Pose3d anemometerPose;
  public: LowPass filter;
  public: gz::transport::Node::Publisher anemometerPub;
  public: std::string anemometerFrameId;
  public: Throttle anemometerThrottle;
};
}  // namespace kingfisher_gazebo

using namespace kingfisher_gazebo;

namespace
{
std::vector<double> ParseList(const std::string &_text, bool &_ok)
{
  std::istringstream in(_text);
  std::vector<double> values;
  double v;
  while (in >> v)
    values.push_back(v);
  _ok = in.eof();
  return values;
}

Duration Period(double _rate)
{
  return std::chrono::duration_cast<Duration>(
    std::chrono::duration<double>(1.0 / _rate));
}

void SetHeader(gz::msgs::Header *_header, const Duration &_simTime,
               const std::string &_frameId)
{
  _header->mutable_stamp()->CopyFrom(gz::msgs::Convert(_simTime));
  auto *data = _header->add_data();
  data->set_key("frame_id");
  data->add_value(_frameId);
}
}  // namespace

//////////////////////////////////////////////////
bool ApparentWindPrivate::ConfigureSail(const sdf::ElementConstPtr &_sdf,
                                        const gz::sim::Model &_model,
                                        gz::sim::EntityComponentManager &_ecm)
{
  const auto linkName = _sdf->Get<std::string>("link_name", "sail_link").first;
  this->sail = gz::sim::Link(_model.LinkByName(_ecm, linkName));
  if (!this->sail.Valid(_ecm))
  {
    gzerr << "ApparentWind: no link [" << linkName << "]" << std::endl;
    return false;
  }
  this->ce = _sdf->Get<gz::math::Vector3d>("center_of_effort",
                                           gz::math::Vector3d::Zero).first;
  this->area = _sdf->Get<double>("area", 0.0).first;
  this->rho = _sdf->Get<double>("air_density", 1.225).first;

  bool okA, okL, okD;
  auto alpha = ParseList(_sdf->Get<std::string>("alpha", "").first, okA);
  auto cl = ParseList(_sdf->Get<std::string>("cl", "").first, okL);
  auto cd = ParseList(_sdf->Get<std::string>("cd", "").first, okD);
  if (!okA || !okL || !okD)
  {
    gzerr << "ApparentWind: <alpha>, <cl> and <cd> must be lists of numbers"
          << std::endl;
    return false;
  }
  try
  {
    this->table = SailTable(alpha, cl, cd);
  }
  catch (const std::invalid_argument &_e)
  {
    gzerr << "ApparentWind: sail table: " << _e.what() << std::endl;
    return false;
  }

  this->sail.EnableVelocityChecks(_ecm);
  this->sailFrameId = _sdf->Get<std::string>("frame_id", linkName).first;
  this->wrenchThrottle.period = Period(_sdf->Get<double>("rate", 20.0).first);
  const auto topic = _sdf->Get<std::string>("topic", "sail/wrench").first;
  this->wrenchPub = this->node.Advertise<gz::msgs::Wrench>(topic);
  gzmsg << "ApparentWind: sail on [" << linkName << "], "
        << this->table.Size() << " table rows, publishing [" << topic << "]"
        << std::endl;
  return true;
}

//////////////////////////////////////////////////
bool ApparentWindPrivate::ConfigureAnemometer(
  const sdf::ElementConstPtr &_sdf, const gz::sim::Entity &_model,
  gz::sim::EntityComponentManager &_ecm)
{
  // The description's fixed-jointed link is a frame after URDF -> SDF
  const auto frameName =
    _sdf->Get<std::string>("frame", "anemometer_link").first;
  const auto *modelSdf = _ecm.Component<gz::sim::components::ModelSdf>(_model);
  const sdf::Frame *frame =
    modelSdf ? modelSdf->Data().FrameByName(frameName) : nullptr;
  if (!frame)
  {
    gzerr << "ApparentWind: no frame [" << frameName << "] in the model"
          << std::endl;
    return false;
  }
  std::string body;
  sdf::Errors errors = frame->ResolveAttachedToBody(body);
  if (errors.empty())
    errors = frame->SemanticPose().Resolve(this->anemometerPose, body);
  if (!errors.empty())
  {
    gzerr << "ApparentWind: cannot resolve frame [" << frameName << "]: "
          << errors.front().Message() << std::endl;
    return false;
  }

  this->anemometerBody = gz::sim::Link(
    gz::sim::Model(_model).LinkByName(_ecm, body));
  if (!this->anemometerBody.Valid(_ecm))
  {
    gzerr << "ApparentWind: no link [" << body << "]" << std::endl;
    return false;
  }
  this->anemometerBody.EnableVelocityChecks(_ecm);
  this->filter = LowPass(_sdf->Get<double>("time_constant", 0.0).first);
  this->anemometerFrameId = _sdf->Get<std::string>("frame_id", frameName).first;
  this->anemometerThrottle.period =
    Period(_sdf->Get<double>("rate", 10.0).first);
  const auto topic =
    _sdf->Get<std::string>("topic", "sensors/anemometer/twist").first;
  this->anemometerPub = this->node.Advertise<gz::msgs::Twist>(topic);
  gzmsg << "ApparentWind: anemometer [" << frameName << "] at ["
        << this->anemometerPose << "] from [" << body << "], publishing ["
        << topic << "]" << std::endl;
  return true;
}

//////////////////////////////////////////////////
ApparentWind::ApparentWind()
  : dataPtr(std::make_unique<ApparentWindPrivate>())
{
}

//////////////////////////////////////////////////
ApparentWind::~ApparentWind() = default;

//////////////////////////////////////////////////
void ApparentWind::Configure(const gz::sim::Entity &_entity,
                             const std::shared_ptr<const sdf::Element> &_sdf,
                             gz::sim::EntityComponentManager &_ecm,
                             gz::sim::EventManager &)
{
  gz::sim::Model model(_entity);
  if (!model.Valid(_ecm))
  {
    gzerr << "ApparentWind must be attached to a model" << std::endl;
    return;
  }

  auto &d = *this->dataPtr;
  d.node.Subscribe(
    _sdf->Get<std::string>("topic_wind_speed", "/vrx/debug/wind/speed").first,
    &ApparentWindPrivate::OnSpeed, &d);
  d.node.Subscribe(
    _sdf->Get<std::string>("topic_wind_direction",
                           "/vrx/debug/wind/direction").first,
    &ApparentWindPrivate::OnDirection, &d);

  if (_sdf->HasElement("sail"))
    d.sailEnabled = d.ConfigureSail(_sdf->FindElement("sail"), model, _ecm);
  if (_sdf->HasElement("anemometer"))
  {
    d.anemometerEnabled = d.ConfigureAnemometer(
      _sdf->FindElement("anemometer"), _entity, _ecm);
  }
}

//////////////////////////////////////////////////
void ApparentWind::PreUpdate(const gz::sim::UpdateInfo &_info,
                             gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  auto &d = *this->dataPtr;
  gz::math::Vector3d wind;
  {
    std::lock_guard<std::mutex> lock(d.mutex);
    const double dir = d.windDirection * GZ_PI / 180.0;
    wind.Set(d.windSpeed * std::cos(dir), d.windSpeed * std::sin(dir), 0.0);
  }

  if (d.sailEnabled)
  {
    const auto pose = d.sail.WorldPose(_ecm);
    const auto vel = d.sail.WorldLinearVelocity(_ecm, d.ce);
    if (pose && vel)
    {
      const auto apparent = pose->Rot().RotateVectorReverse(wind - *vel);
      const auto force = SailForce(apparent, d.table, d.area, d.rho);
      d.sail.AddWorldWrench(_ecm, pose->Rot().RotateVector(force),
                            gz::math::Vector3d::Zero, d.ce);
      if (d.wrenchThrottle.Ready(_info.simTime))
      {
        gz::msgs::Wrench msg;
        SetHeader(msg.mutable_header(), _info.simTime, d.sailFrameId);
        gz::msgs::Set(msg.mutable_force(), force);
        gz::msgs::Set(msg.mutable_torque(), d.ce.Cross(force));
        d.wrenchPub.Publish(msg);
      }
    }
  }

  if (d.anemometerEnabled)
  {
    const auto pose = d.anemometerBody.WorldPose(_ecm);
    const auto vel = d.anemometerBody.WorldLinearVelocity(
      _ecm, d.anemometerPose.Pos());
    if (pose && vel)
    {
      auto apparent = (pose->Rot() * d.anemometerPose.Rot())
        .RotateVectorReverse(wind - *vel);
      apparent.Z(0.0);
      const auto &reading = d.filter.Update(
        apparent, std::chrono::duration<double>(_info.dt).count());
      if (d.anemometerThrottle.Ready(_info.simTime))
      {
        gz::msgs::Twist msg;
        SetHeader(msg.mutable_header(), _info.simTime, d.anemometerFrameId);
        gz::msgs::Set(msg.mutable_linear(), reading);
        d.anemometerPub.Publish(msg);
      }
    }
  }
}

GZ_ADD_PLUGIN(kingfisher_gazebo::ApparentWind,
              gz::sim::System,
              kingfisher_gazebo::ApparentWind::ISystemConfigure,
              kingfisher_gazebo::ApparentWind::ISystemPreUpdate)
