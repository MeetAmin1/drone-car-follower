#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <sdf/Element.hh>

namespace drone_assessment
{
constexpr double kPi = 3.14159265358979323846;

class FigureEightMotion final:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /*_eventMgr*/) override
  {
    this->model = gz::sim::Model(_entity);
    if (!this->model.Valid(_ecm))
    {
      gzerr << "FigureEightMotion must be attached to a model.\n";
      return;
    }

    this->configured = true;
    this->ReadParameter(_sdf, "center_x", this->centerX);
    this->ReadParameter(_sdf, "center_y", this->centerY);
    this->ReadParameter(_sdf, "height", this->height);
    this->ReadParameter(_sdf, "amplitude_x", this->amplitudeX);
    this->ReadParameter(_sdf, "amplitude_y", this->amplitudeY);
    this->ReadParameter(_sdf, "period_s", this->periodS);

    if (this->periodS <= 0.0)
    {
      gzerr << "FigureEightMotion period_s must be positive.\n";
      this->configured = false;
    }
  }

  void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->configured || _info.paused)
    {
      return;
    }

    const double t = std::chrono::duration<double>(_info.simTime).count();
    const double omega = 2.0 * kPi / this->periodS;
    const double phase = omega * t;

    const double x = this->centerX + this->amplitudeX * std::sin(phase);
    const double y = this->centerY + this->amplitudeY * std::sin(2.0 * phase);
    const double dx = this->amplitudeX * omega * std::cos(phase);
    const double dy = 2.0 * this->amplitudeY * omega * std::cos(2.0 * phase);
    const double yaw = std::atan2(dy, dx);

    this->model.SetWorldPoseCmd(
      _ecm,
      gz::math::Pose3d(x, y, this->height, 0.0, 0.0, yaw));
  }

private:
  template<typename T>
  void ReadParameter(
    const std::shared_ptr<const sdf::Element> &_sdf,
    const std::string &_name,
    T &_value)
  {
    if (_sdf && _sdf->HasElement(_name))
    {
      _value = _sdf->Get<T>(_name);
    }
  }

  gz::sim::Model model{gz::sim::kNullEntity};
  bool configured{false};
  double centerX{10.0};
  double centerY{0.0};
  double height{0.5};
  double amplitudeX{10.0};
  double amplitudeY{6.0};
  double periodS{28.0};
};
}  // namespace drone_assessment

GZ_ADD_PLUGIN(
  drone_assessment::FigureEightMotion,
  gz::sim::System,
  drone_assessment::FigureEightMotion::ISystemConfigure,
  drone_assessment::FigureEightMotion::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  drone_assessment::FigureEightMotion,
  "drone_assessment::FigureEightMotion")
