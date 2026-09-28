// Copyright 2025 Luis Batista
//
// Licensed under the BSD License; see the package.xml for details.

// Sail aerodynamics and the anemometer filter used by ApparentWind.
// Header-only and free of gz-sim, so it can be unit tested.

#ifndef KINGFISHER_GAZEBO_WINDMODEL_HH_
#define KINGFISHER_GAZEBO_WINDMODEL_HH_

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include <gz/math/Vector3.hh>

namespace kingfisher_gazebo
{
/// \brief Lift and drag coefficients of a symmetric profile over 0..180 deg.
/// C_L is mirrored as odd and C_D as even for negative angles.
class SailTable
{
  public: SailTable() = default;

  /// \param[in] _alphaDeg Angles of attack in degrees, ascending, 0 to 180.
  public: SailTable(std::vector<double> _alphaDeg, std::vector<double> _cl,
                    std::vector<double> _cd)
    : alpha(std::move(_alphaDeg)), cl(std::move(_cl)), cd(std::move(_cd))
  {
    if (this->alpha.size() < 2 || this->cl.size() != this->alpha.size() ||
        this->cd.size() != this->alpha.size())
      throw std::invalid_argument("alpha, cl and cd need the same size, >= 2");
    if (this->alpha.front() != 0.0 || this->alpha.back() != 180.0 ||
        !std::is_sorted(this->alpha.begin(), this->alpha.end()))
      throw std::invalid_argument("alpha must ascend from 0 to 180 deg");
  }

  /// \param[in] _alpha Angle of attack in radians, in [-pi, pi].
  public: double Cl(double _alpha) const
  {
    const double c = this->Interp(this->cl, std::abs(_alpha));
    return _alpha < 0 ? -c : c;
  }

  public: double Cd(double _alpha) const
  {
    return this->Interp(this->cd, std::abs(_alpha));
  }

  public: size_t Size() const
  {
    return this->alpha.size();
  }

  private: double Interp(const std::vector<double> &_y, double _absAlpha) const
  {
    const double deg = std::clamp(_absAlpha * 180.0 / M_PI, 0.0, 180.0);
    auto hi = std::upper_bound(this->alpha.begin(), this->alpha.end(), deg);
    if (hi == this->alpha.end())
      return _y.back();
    const auto i = static_cast<size_t>(hi - this->alpha.begin());
    const double t = (deg - this->alpha[i - 1]) /
                     (this->alpha[i] - this->alpha[i - 1]);
    return _y[i - 1] + t * (_y[i] - _y[i - 1]);
  }

  private: std::vector<double> alpha;
  private: std::vector<double> cl;
  private: std::vector<double> cd;
};

/// \brief Angle of attack: from the chord's nose (+x) to where the wind
/// comes from, counter-clockwise. 0 with the nose into the wind.
/// \param[in] _v Apparent wind in the sail frame, pointing where it goes.
inline double AngleOfAttack(const gz::math::Vector3d &_v)
{
  return std::atan2(-_v.Y(), -_v.X());
}

/// \brief Sail force in the sail frame: F = q A [C_D u + C_L (z x u)], with
/// u the unit apparent wind in the xy plane. z is ignored.
/// \param[in] _v Apparent wind in the sail frame, pointing where it goes.
inline gz::math::Vector3d SailForce(const gz::math::Vector3d &_v,
                                    const SailTable &_table,
                                    double _area, double _rho)
{
  const double speed = std::hypot(_v.X(), _v.Y());
  if (speed < 1e-9)
    return gz::math::Vector3d::Zero;
  const double ux = _v.X() / speed;
  const double uy = _v.Y() / speed;
  const double alpha = AngleOfAttack(_v);
  const double cl = _table.Cl(alpha);
  const double cd = _table.Cd(alpha);
  const double qa = 0.5 * _rho * speed * speed * _area;
  return {qa * (cd * ux - cl * uy), qa * (cd * uy + cl * ux), 0.0};
}

/// \brief First-order low-pass, y += dt / (tau + dt) (x - y), starting from
/// the first sample. A pass-through when tau <= 0.
class LowPass
{
  public: explicit LowPass(double _tau = 0.0) : tau(_tau) {}

  public: const gz::math::Vector3d &Update(const gz::math::Vector3d &_x,
                                           double _dt)
  {
    if (!this->started || this->tau <= 0.0)
    {
      this->y = _x;
      this->started = true;
    }
    else if (_dt > 0.0)
    {
      this->y += (_x - this->y) * (_dt / (this->tau + _dt));
    }
    return this->y;
  }

  private: double tau;
  private: bool started{false};
  private: gz::math::Vector3d y;
};
}  // namespace kingfisher_gazebo

#endif  // KINGFISHER_GAZEBO_WINDMODEL_HH_
