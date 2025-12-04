#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "blvr_diffbot_hardware/blvr_comunicator.h"

using std::placeholders::_1;
using std::placeholders::_2;

class BlvrStopServer : public rclcpp::Node
{
public:
  BlvrStopServer()
  : Node("blvr_stop_server")
  {
    // Init communicator (adjust your device, baud, etc.)
    comm_ = std::make_shared<blvr_diffbot_hardware::BlvrComunicator>();
    auto rc = comm_->openDevice("/dev/ttyUSB1", 230400, 'E', 8, 1);  // ⚠️ adjust to your setup
    if (rc != blvr_diffbot_hardware::BlvrComunicator::return_type::SUCCESS) {
      RCLCPP_FATAL(get_logger(), "Failed to open BLV-R device!");
      rclcpp::shutdown();
      return;
    }

    // Create service
    srv_ = this->create_service<std_srvs::srv::Trigger>(
      "emergency_stop",
      std::bind(&BlvrStopServer::handle_stop, this, _1, _2));

    RCLCPP_INFO(get_logger(), "Emergency stop service ready on /emergency_stop");
  }

private:
  void handle_stop(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    int rc = comm_->immediateStop();
    if (rc == 0) {
      res->success = true;
      res->message = "Immediate stop broadcasted (mode=32)";
      RCLCPP_WARN(this->get_logger(), "Immediate stop executed!");
    } else {
      res->success = false;
      res->message = "Failed to send immediate stop";
      RCLCPP_ERROR(this->get_logger(), "Immediate stop failed rc=%d", rc);
    }
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
  std::shared_ptr<blvr_diffbot_hardware::BlvrComunicator> comm_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BlvrStopServer>());
  rclcpp::shutdown();
  return 0;
}
