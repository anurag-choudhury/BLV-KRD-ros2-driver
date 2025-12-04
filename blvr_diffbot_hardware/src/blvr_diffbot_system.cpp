#include "blvr_diffbot_hardware/blvr_diffbot_system.hpp"

#include <cmath>
#include <memory>
#include <utility>

#include "pluginlib/class_list_macros.hpp"


namespace blvr_diffbot_hardware
{

hardware_interface::CallbackReturn BlvrDiffbotSystemHardware::on_init(const hardware_interface::HardwareInfo &info)
{
  RCLCPP_INFO(rclcpp::get_logger("BlvrDiffbotSystemHardware"), "Initializing hardware interface...");

  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("BlvrDiffbotSystemHardware"), "Parent on_init failed");
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    hw_start_sec_        = std::stod(info_.hardware_parameters.at("example_param_hw_start_duration_sec"));
    hw_stop_sec_         = std::stod(info_.hardware_parameters.at("example_param_hw_stop_duration_sec"));
    device_name_         = info_.hardware_parameters.at("device_name");
    gear_ratio_          = std::stod(info_.hardware_parameters.at("gear_ratio"));
    encoder_resolution_  = std::stod(info_.hardware_parameters.at("encoder_resolution"));
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("BlvrDiffbotSystemHardware"), "Parameter parse error: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_positions_.assign(info_.joints.size(), 0.0);
  hw_velocities_.assign(info_.joints.size(), 0.0);
  hw_efforts_.assign(info_.joints.size(), 0.0);
  hw_commands_.assign(info_.joints.size(), 0.0);
  prev_step_.assign(info_.joints.size(), 0);

  serial_port_ = std::make_shared<BlvrComunicator>();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> BlvrDiffbotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT,   &hw_efforts_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> BlvrDiffbotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn
BlvrDiffbotSystemHardware::on_activate(const rclcpp_lifecycle::State &)
{
  // Optional warmup before starting
  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(hw_start_sec_)));

  for (size_t i = 0; i < hw_positions_.size(); i++) {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_commands_[i]   = 0.0;
    prev_step_[i]     = 0;
  }

  // Open serial
  if (serial_port_->openDevice(device_name_, 230400, 'E', 8, 1)
      != BlvrComunicator::return_type::SUCCESS) {
    RCLCPP_FATAL(rclcpp::get_logger("BlvrDiffbotSystemHardware"),
                 "Failed to open device %s.", device_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ---- Retry-excite both motors ----
  constexpr int kExciteRetries = 5;
  constexpr std::chrono::milliseconds kBaseBackoff{50};
  constexpr std::chrono::milliseconds kMaxBackoff{200};

  auto try_excite = [&](int motor_id) -> bool {
    std::chrono::milliseconds backoff = kBaseBackoff;
    for (int attempt = 1; attempt <= kExciteRetries; ++attempt) {
      const int rc = serial_port_->setExcitation(motor_id);
      if (rc >= 0) {
        RCLCPP_INFO(rclcpp::get_logger("BlvrDiffbotSystemHardware"),
                    "Motor %d excited (attempt %d, rc=%d).",
                    motor_id, attempt, rc);
        return true;
      }

      RCLCPP_WARN(rclcpp::get_logger("BlvrDiffbotSystemHardware"),
                  "Excitation failed for motor %d (attempt %d/%d, rc=%d).%s",
                  motor_id, attempt, kExciteRetries, rc,
                  attempt < kExciteRetries ? " Retrying..." : " No more retries.");

      // Optional: drain/flush between attempts if your communicator supports it
      // serial_port_->flush(); serial_port_->drain();

      if (attempt < kExciteRetries) {
        rclcpp::sleep_for(backoff);
        backoff = std::min(backoff * 2, kMaxBackoff);
      }
    }
    return false;
  };

  bool excite_ok = true;
  for (int motor_id = 1; motor_id <= 2; ++motor_id) {
    if (!try_excite(motor_id)) {
      excite_ok = false;
      RCLCPP_ERROR(rclcpp::get_logger("BlvrDiffbotSystemHardware"),
                   "Motor %d failed to excite after %d attempts.",
                   motor_id, kExciteRetries);
    }
  }

  if (!excite_ok) {
    // If you prefer to still start IO and let recoveries handle it, change to WARNING and continue.
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Start I/O thread after successful excitation
  io_running_.store(true);
  io_thread_ = std::thread([this]() { this->ioLoop(); });

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn BlvrDiffbotSystemHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Stop I/O thread
  io_running_.store(false);
  if (io_thread_.joinable()) io_thread_.join();

  // Graceful stop
  if (serial_port_) {
    serial_port_->writeSpeedCommand(1, 0);
    serial_port_->writeSpeedCommand(2, 0);
    serial_port_->closeDevice();
  }

  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(hw_stop_sec_)));
  return hardware_interface::CallbackReturn::SUCCESS;
}

void BlvrDiffbotSystemHardware::ioLoop()
{
  auto last_trigger = std::chrono::steady_clock::now() - trigger_min_period_;

  while (io_running_.load()) {
    const auto loop_start = std::chrono::steady_clock::now();

    // 1) Poll both motors quickly
    for (int i = 0; i < 2; ++i) {
      const int motor_id = i + 1;
      int step=0, rpm=0, tq=0;
      if (serial_port_->readStepRpmTorque(motor_id, &step, &rpm, &tq) == 0) {
        std::lock_guard<std::mutex> lk(cache_mtx_);
        cached_[i].step   = step;
        cached_[i].rpm    = rpm;
        cached_[i].torque = tq;
        cached_[i].stamp  = rclcpp::Clock().now();
        cached_[i].valid  = true;
      } else {
        std::lock_guard<std::mutex> lk(cache_mtx_);
        cached_[i].valid = false;
      }
    }

    // 2) Apply pending commands (coalesced)
    bool any_changed = false;
    {
      std::lock_guard<std::mutex> lk(cmd_mtx_);
      for (int i = 0; i < 2; ++i) {
        if (pending_rpm_[i].has_value()) {
          const int rpm = pending_rpm_[i].value();
          if (rpm != last_sent_rpm_[i]) {
            if (serial_port_->writeSpeedCommand(i+1, rpm) == 0) {
              last_sent_rpm_[i] = rpm;
              any_changed = true;
            }
          }
          pending_rpm_[i].reset();
        }
      }
    }

    // 3) Single trigger (rate-limited) — broadcastTrigger uses MSW-first
    if (any_changed) {
      auto now = std::chrono::steady_clock::now();
      if (now - last_trigger >= trigger_min_period_) {
        serial_port_->broadcastTrigger(102, 10000);
        last_trigger = now;
      }
    }

    // 4) Keep stable period
    const auto elapsed = std::chrono::steady_clock::now() - loop_start;
    if (elapsed < poll_period_) {
      std::this_thread::sleep_for(poll_period_ - elapsed);
    }
  }
}

hardware_interface::return_type BlvrDiffbotSystemHardware::read(
    const rclcpp::Time &, const rclcpp::Duration &)
{
  std::array<CachedMotorState, 2> snap;
  {
    std::lock_guard<std::mutex> lk(cache_mtx_);
    snap = cached_;
  }

  for (size_t i = 0; i < hw_commands_.size(); i++) {
    const int motor_direction = (i % 2 == 0) ? 1 : -1;

    const int step   = snap[i].valid ? snap[i].step   : prev_step_[i];
    const int rpm    = snap[i].valid ? snap[i].rpm    : 0;
    const int torque = snap[i].valid ? snap[i].torque : 0;

    const int diff_step = step - prev_step_[i];
    prev_step_[i] = step;

    // BLV reports motor-side → convert to wheel-side
    hw_positions_[i] += static_cast<double>(motor_direction * diff_step)
                        * 2.0 * M_PI / (gear_ratio_ * encoder_resolution_);
    hw_velocities_[i] = static_cast<double>(motor_direction * rpm)
                        * 2.0 * M_PI / (gear_ratio_ * 60.0);

    // Torque scaling (keep as before, adjust if needed)
    hw_efforts_[i] = static_cast<double>(motor_direction * torque)
                     / 1000.0 * BLVR_RATED_TORQUE * gear_ratio_;
  }

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type BlvrDiffbotSystemHardware::write(
    const rclcpp::Time &, const rclcpp::Duration &)
{
  // Always initialize with zeros
  std::array<int, 2> rpm_cmds{0, 0};

  for (size_t i = 0; i < hw_commands_.size(); ++i) {
    const int motor_direction = (i % 2 == 0) ? 1 : -1;

    // Wheel rad/s → motor rpm (multiply by gear_ratio_)
    double motor_rpm_d = motor_direction *
      (hw_commands_[i] * 60.0 * gear_ratio_ / (2.0 * M_PI));

    // Clamp for safety (adjust MAX_RPM to your motor spec)
    const int MAX_ABS_RPM = 3000;
    int rpm = static_cast<int>(std::llround(motor_rpm_d));
    if (rpm >  MAX_ABS_RPM) rpm =  MAX_ABS_RPM;
    if (rpm < -MAX_ABS_RPM) rpm = -MAX_ABS_RPM;

    // ✅ now actually store it
    rpm_cmds[i] = rpm;
  }

  // Post to mailbox
  {
    std::lock_guard<std::mutex> lk(cmd_mtx_);
    for (int i = 0; i < 2; ++i) {
      pending_rpm_[i] = rpm_cmds[i];
    }
  }

  return hardware_interface::return_type::OK;
}


} // namespace blvr_diffbot_hardware

PLUGINLIB_EXPORT_CLASS(blvr_diffbot_hardware::BlvrDiffbotSystemHardware, hardware_interface::SystemInterface)
