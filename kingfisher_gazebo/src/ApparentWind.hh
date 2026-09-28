// Copyright 2025 Luis Batista
//
// Licensed under the BSD License; see the package.xml for details.

#ifndef KINGFISHER_GAZEBO_APPARENTWIND_HH_
#define KINGFISHER_GAZEBO_APPARENTWIND_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace kingfisher_gazebo
{
class ApparentWindPrivate;

/// \brief Sail lift and drag and a 2D anemometer, both from the apparent
/// wind: vrx::USVWind's true wind minus the velocity of the point.
///
/// Parameters are in kingfisher_gazebo_plugins.xacro (kingfisher_wind_plugin).
/// Each of <sail> and <anemometer> is optional.
class ApparentWind
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
  public: ApparentWind();

  public: ~ApparentWind() override;

  public: void Configure(const gz::sim::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         gz::sim::EntityComponentManager &_ecm,
                         gz::sim::EventManager &_eventMgr) override;

  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                         gz::sim::EntityComponentManager &_ecm) override;

  private: std::unique_ptr<ApparentWindPrivate> dataPtr;
};
}  // namespace kingfisher_gazebo

#endif  // KINGFISHER_GAZEBO_APPARENTWIND_HH_
