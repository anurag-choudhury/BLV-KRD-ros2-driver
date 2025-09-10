#include "blvr_diffbot_hardware/blvr_diffbot_system.hpp"
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <future>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace blvr_diffbot_hardware
{

  constexpr double BLVR_RATED_TORQUE = 0.319; // Nm

  hardware_interface::CallbackReturn BlvrDiffbotSystemHardware::on_init(const hardware_interface::HardwareInfo &info)
  {
    RCLCPP_INFO(rclcpp::get_logger("BlvrDiffbotSystemHardware"), "Initializing hardware interface...");
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      RCLCPP_ERROR(rclcpp::get_logger("BlvrDiffbotSystemHardware"), "Parent on_init failed");
      return hardware_interface::CallbackReturn::ERROR;
    }

    try
    {
      hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
      hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
      device_name_ = info_.hardware_parameters["device_name"];
      gear_ratio_ = stod(info_.hardware_parameters["gear_ratio"]);
      encoder_resolution_ = stod(info_.hardware_parameters["encoder_resolution"]);
    }
    catch (const std::exception &e)
    {
      RCLCPP_FATAL(rclcpp::get_logger("BlvrDiffbotSystemHardware"), "Parameter parse error: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    prev_step_.resize(info_.joints.size());

    serial_port_ = std::make_shared<BlvrComunicator>();
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> BlvrDiffbotSystemHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> BlvrDiffbotSystemHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    }
    return command_interfaces;
  }

  hardware_interface::CallbackReturn BlvrDiffbotSystemHardware::on_activate(const rclcpp_lifecycle::State &)
  {
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(hw_start_sec_)));

    for (auto i = 0u; i < hw_positions_.size(); i++)
    {
      if (std::isnan(hw_positions_[i]))
      {
        hw_positions_[i] = 0;
        hw_velocities_[i] = 0;
        hw_commands_[i] = 0;
      }
    }

    if (serial_port_->openDevice(device_name_, 230400, 'E', 8, 1) != BlvrComunicator::return_type::SUCCESS)
    {
      RCLCPP_FATAL(rclcpp::get_logger("BlvrDiffbotSystemHardware"), "Failed to open device.");
      return hardware_interface::CallbackReturn::ERROR;
    }

    for (int motor_id = 1; motor_id <= 2; ++motor_id) {
      if (serial_port_->setExcitation(motor_id) != 2)
        RCLCPP_WARN(rclcpp::get_logger("BlvrDiffbotSystemHardware"), "Failed to excite motor %d", motor_id);
      else
        RCLCPP_INFO(rclcpp::get_logger("BlvrDiffbotSystemHardware"), "Motor %d excited successfully", motor_id);
    }


    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BlvrDiffbotSystemHardware::on_deactivate(const rclcpp_lifecycle::State &)
  {
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(hw_stop_sec_)));

    serial_port_->writeSpeedCommand(1, 0);
    serial_port_->writeSpeedCommand(2, 0);
    serial_port_->closeDevice();
    return hardware_interface::CallbackReturn::SUCCESS;
  }


hardware_interface::return_type BlvrDiffbotSystemHardware::read(
    const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < hw_commands_.size(); i++)
  {
    int motor_id = static_cast<int>(i) + 1;
    int motor_direction = (i % 2 == 0) ? 1 : -1;

    int step = 0, rpm = 0, torque = 0;
    if (serial_port_->readStepRpmTorque(motor_id, &step, &rpm, &torque) != 0) {
      RCLCPP_WARN(rclcpp::get_logger("BlvrDiffbotSystemHardware"),
                  "Failed to read (step,rpm,torque) from motor %d", motor_id);
      
      continue;
    }
    // RCLCPP_INFO(rclcpp::get_logger("BlvrDiffbotSystemHardware"),
    //               "toque motor %d", torque);

    // (Unchanged) your original logic below
    int diff_step = step - prev_step_[i];
    prev_step_[i] = step;

    hw_positions_[i] += static_cast<double>(motor_direction * diff_step)
                        * 2.0 * M_PI / gear_ratio_ / encoder_resolution_;
    hw_velocities_[i] = static_cast<double>(motor_direction * rpm)
                        * 2.0 * M_PI / gear_ratio_ / 60.0;
    hw_efforts_[i]    = static_cast<double>(motor_direction * torque)
                        / 1000.0 * BLVR_RATED_TORQUE * gear_ratio_;

    // Optional debug:
    // RCLCPP_INFO(rclcpp::get_logger("BlvrDiffbotSystemHardware"),
    //             "RPM %d STEP %d TORQUE %d", rpm, step, torque);
  }

  return hardware_interface::return_type::OK;
}



  hardware_interface::return_type
BlvrDiffbotSystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Initialize last_sent_rpm on first call or if joint count changes
  static std::vector<int> last_sent_rpm(hw_commands_.size(),
                                        std::numeric_limits<int>::min());

  struct Cmd { int motor_id; int rpm; };
  std::vector<Cmd> cmds(hw_commands_.size());

  // 1) Compute desired RPMs for this tick
  for (size_t i = 0; i < hw_commands_.size(); ++i) {
    const int motor_id = static_cast<int>(i) + 1;
    const int motor_direction = (i % 2 == 0) ? 1 : -1;  // left +, right -
    const int rpm = motor_direction *
                    static_cast<int>(std::llround(hw_commands_[i] * 60.0 *
                                                  gear_ratio_ / (2.0 * M_PI)));
    cmds[i] = {motor_id, rpm};
  }

  // 2) Stage only motors whose RPM changed
  bool any_changed = false;
  for (size_t i = 0; i < cmds.size(); ++i) {
    if (cmds[i].rpm != last_sent_rpm[i]) {
      const int rc = serial_port_->writeSpeedCommand(cmds[i].motor_id, cmds[i].rpm);
      if (rc == 0) {
        RCLCPP_INFO(rclcpp::get_logger("BlvrDiffbotSystemHardware"),
                    "Staged rpm %d for motor %d", cmds[i].rpm, cmds[i].motor_id);
        last_sent_rpm[i] = cmds[i].rpm;
        any_changed = true;
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("BlvrDiffbotSystemHardware"),
                     "Failed to stage rpm %d for motor %d",
                     cmds[i].rpm, cmds[i].motor_id);
        any_changed = false;
        break;
      }
    }
  }

  // 3) If anything changed, broadcast one trigger to both motors
  if (any_changed) {
    RCLCPP_INFO(rclcpp::get_logger("BlvrDiffbotSystemHardware"),
                "Broadcasting trigger pulse (reg 102) to latch staged speeds");

    serial_port_->broadcastTrigger(102, 10000);

    RCLCPP_INFO(rclcpp::get_logger("BlvrDiffbotSystemHardware"),
                "Broadcast trigger pulse sent");
  }

  return hardware_interface::return_type::OK;
}



} // namespace blvr_diffbot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(blvr_diffbot_hardware::BlvrDiffbotSystemHardware, hardware_interface::SystemInterface)
