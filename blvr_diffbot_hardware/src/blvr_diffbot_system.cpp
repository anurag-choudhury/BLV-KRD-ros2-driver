#include "blvr_diffbot_hardware/blvr_diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace blvr_diffbot_hardware
{

  constexpr double BLVR_RATED_TORQUE = 0.64; // Nm

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

    RCLCPP_INFO(rclcpp::get_logger("BlvrDiffbotSystemHardware"), "Parameters loaded");

    hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    prev_step_.resize(info_.joints.size());

    serial_port_ = std::make_shared<BlvrComunicator>();
    RCLCPP_INFO(rclcpp::get_logger("BlvrDiffbotSystemHardware"), "Hardware interface initialized");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> BlvrDiffbotSystemHardware::export_state_interfaces()
  {
    RCLCPP_INFO(rclcpp::get_logger("BlvrDiffbotSystemHardware"), "Exporting state interfaces...");
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
    RCLCPP_INFO(rclcpp::get_logger("BlvrDiffbotSystemHardware"), "Exporting command interfaces...");
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    }
    return command_interfaces;
  }

  hardware_interface::CallbackReturn BlvrDiffbotSystemHardware::on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("BlvrDiffbotSystemHardware"), "Activating hardware...");
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

    RCLCPP_INFO(rclcpp::get_logger("BlvrDiffbotSystemHardware"), "Opening serial device: %s", device_name_.c_str());

    if (serial_port_->openDevice(device_name_, 9600, 'N', 8, 1) != BlvrComunicator::return_type::SUCCESS)
    {
      RCLCPP_FATAL(rclcpp::get_logger("BlvrDiffbotSystemHardware"), "Failed to open device.");
      return hardware_interface::CallbackReturn::ERROR;
    }
    // Excite both motors
    for (int motor_id = 1; motor_id <= 2; motor_id++)
    {
      if (motor_id == 2)
        continue;
      if (serial_port_->setExcitation(motor_id) != 2)

      {
        RCLCPP_WARN(rclcpp::get_logger("BlvrDiffbotSystemHardware"), "Failed to excite motor %d", motor_id);
      }
      else
      {
        RCLCPP_INFO(rclcpp::get_logger("BlvrDiffbotSystemHardware"), "Motor %d excited successfully", motor_id);
      }
      usleep(200000);
      serial_port_->writeSpeedCommand(motor_id, 150);
    }
    RCLCPP_INFO(rclcpp::get_logger("BlvrDiffbotSystemHardware"), "System successfully started");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BlvrDiffbotSystemHardware::on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("BlvrDiffbotSystemHardware"), "Deactivating hardware...");
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(hw_stop_sec_)));

    serial_port_->closeDevice();
    RCLCPP_INFO(rclcpp::get_logger("BlvrDiffbotSystemHardware"), "System successfully stopped");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type BlvrDiffbotSystemHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
  {
    RCLCPP_INFO(rclcpp::get_logger("BlvrDiffbotSystemHardware"), "Reading motor states...");
    for (size_t i = 0; i < hw_commands_.size(); i++)
    {
      if (i == 1)
        continue;
      int motor_id = i + 1;
      int motor_direction = (i % 2 == 0) ? 1 : -1;

      int step = 0, rpm = 0, torque = 0;
      if (serial_port_->readStep(motor_id, &step) != 0)
        RCLCPP_WARN(rclcpp::get_logger("BlvrDiffbotSystemHardware"), "Failed to read step from motor %d", motor_id);
      if (serial_port_->readRpm(motor_id, &rpm) != 0)
        RCLCPP_WARN(rclcpp::get_logger("BlvrDiffbotSystemHardware"), "Failed to read rpm from motor %d", motor_id);
      if (serial_port_->readTorque(motor_id, &torque) != 0)
        RCLCPP_WARN(rclcpp::get_logger("BlvrDiffbotSystemHardware"), "Failed to read torque from motor %d", motor_id);
      RCLCPP_INFO(rclcpp::get_logger("BlvrDiffbotSystemHardware"), "RPM %d STEP %d", rpm, step);
      int diff_step = step - prev_step_[i];
      prev_step_[i] = step;

      hw_positions_[i] += static_cast<double>(motor_direction * diff_step) * 2.0 * M_PI / gear_ratio_ / encoder_resolution_;
      hw_velocities_[i] = static_cast<double>(motor_direction * rpm) * 2.0 * M_PI / gear_ratio_ / 60.0;
      hw_efforts_[i] = static_cast<double>(motor_direction * torque) / 1000.0 * BLVR_RATED_TORQUE * gear_ratio_;
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type BlvrDiffbotSystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
  {
    RCLCPP_INFO(rclcpp::get_logger("BlvrDiffbotSystemHardware"), "Writing motor commands...");
    for (size_t i = 0; i < hw_commands_.size(); i++)
    {
      if (i == 1)
        continue;
      int motor_id = i + 1;
      int motor_direction = (i % 2 == 0) ? 1 : -1;
      int rpm = motor_direction * static_cast<int>(hw_commands_[i] * 60.0 * gear_ratio_ / (2.0 * M_PI));

      if (serial_port_->writeSpeedCommand(motor_id, rpm) != 0)
      {
        RCLCPP_WARN(rclcpp::get_logger("BlvrDiffbotSystemHardware"), "Failed to send speed to motor %d", motor_id);
      }
    }

    return hardware_interface::return_type::OK;
  }

} // namespace blvr_diffbot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(blvr_diffbot_hardware::BlvrDiffbotSystemHardware, hardware_interface::SystemInterface)