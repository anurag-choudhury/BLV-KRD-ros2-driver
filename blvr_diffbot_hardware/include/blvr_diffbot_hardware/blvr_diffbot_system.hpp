#pragma once

#include <atomic>
#include <array>
#include <optional>
#include <thread>
#include <vector>
#include <limits>
#include <mutex>
#include <chrono>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "blvr_diffbot_hardware/blvr_comunicator.h"


namespace blvr_diffbot_hardware
{

class BlvrDiffbotSystemHardware final : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ---- Parameters ----
  double hw_start_sec_{0.0};
  double hw_stop_sec_{0.0};
  std::string device_name_{"/dev/ttyUSB0"};
  double gear_ratio_{1.0};
  double encoder_resolution_{1000.0};

  static constexpr double BLVR_RATED_TORQUE = 0.319; // Nm

  // ---- State/Command storage ----
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  std::vector<double> hw_commands_;
  std::vector<int>    prev_step_;

  // ---- Modbus communicator ----
  std::shared_ptr<BlvrComunicator> serial_port_;


  // ---- Non-blocking cache & I/O thread ----
  struct CachedMotorState {
    int step{0}, rpm{0}, torque{0};
    rclcpp::Time stamp{};
    bool valid{false};
  };

  std::array<CachedMotorState, 2> cached_;
  std::array<int, 2> last_sent_rpm_{ {std::numeric_limits<int>::min(), std::numeric_limits<int>::min()} };

  std::mutex cache_mtx_;
  std::mutex cmd_mtx_;
  std::array<std::optional<int>, 2> pending_rpm_;

  std::atomic<bool> io_running_{false};
  std::thread io_thread_;
  std::chrono::milliseconds poll_period_{5};          // ~200 Hz polling
  std::chrono::milliseconds trigger_min_period_{20};  // ≤50 Hz trigger

  void ioLoop();
};

} // namespace blvr_diffbot_hardware
