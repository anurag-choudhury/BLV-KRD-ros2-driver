#ifndef BLVR_DIFFBOT_SYSTEM_HPP_
#define BLVR_DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "blvr_diffbot_hardware/visibility_control.h"
#include "blvr_diffbot_hardware/blvr_comunicator.h"

namespace blvr_diffbot_hardware
{

  class BlvrDiffbotSystemHardware : public hardware_interface::SystemInterface
  {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(BlvrDiffbotSystemHardware)

    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    std::shared_ptr<BlvrComunicator> serial_port_;
    std::string device_name_;
    double gear_ratio_;
    double encoder_resolution_;
    double hw_start_sec_;
    double hw_stop_sec_;

    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    std::vector<double> hw_efforts_;
    std::vector<double> hw_commands_;

    std::vector<int> prev_step_;
  };

} // namespace blvr_diffbot_hardware

#endif // BLVR_DIFFBOT_SYSTEM_HPP_
