#include "blvr_diffbot_hardware/blvr_comunicator.h"

#include <iostream>
#include <unistd.h>

namespace blvr_diffbot_hardware
{

  BlvrComunicator::BlvrComunicator() : ctx_(nullptr) {}

  BlvrComunicator::~BlvrComunicator()
  {
    if (ctx_)
    {
      modbus_close(ctx_);
      modbus_free(ctx_);
    }
  }

  BlvrComunicator::return_type BlvrComunicator::openDevice(
      const std::string &device, int baud, char parity,
      int data_bit, int stop_bit)
  {
    ctx_ = modbus_new_rtu(device.c_str(), baud, parity, data_bit, stop_bit);
    modbus_set_response_timeout(ctx_, 1, 500000); // 1.5 seconds
    if (ctx_ == NULL)
    {
      std::cerr << "Unable to allocate libmodbus context" << std::endl;
      exit(1);
    }

    if (modbus_connect(ctx_) == -1)
    {
      std::cerr << "Connection failed: " << modbus_strerror(errno) << std::endl;
      modbus_free(ctx_);
      exit(1);
    }

    std::cout << "Exited Initialized function" << std::endl;
    return return_type::SUCCESS;
  }

  void BlvrComunicator::closeDevice()
  {
    if (ctx_)
    {
      modbus_close(ctx_);
      modbus_free(ctx_);
      ctx_ = nullptr;
    }
  }

  int BlvrComunicator::setExcitation(int motor_id)
  {
    int rc = writeRegister(motor_id, 0x007C, 1);
    if (rc != 2)
    {
      std::cerr << "[Excitation] Failed to excite motor " << motor_id << std::endl;
    }
    else
    {
      std::cout << "[Excitation] Motor " << motor_id << " excited successfully" << rc << std::endl;
    }
    return rc;
  }

  int BlvrComunicator::readStep(int motor_id, int *step)
  {
    return readRegister(motor_id, 0x00CC, step);
  }

  int BlvrComunicator::readRpm(int motor_id, int *rpm)
  {
    return readRegister(motor_id, 0x00CE, rpm);
  }

  int BlvrComunicator::readTorque(int motor_id, int *torque)
  {
    return readRegister(motor_id, 0x00D6, torque);
  }

  int BlvrComunicator::writeSpeedCommand(int motor_id, int rpm)
  {
    if (!ctx_)
      return -1;

    modbus_set_slave(ctx_, motor_id);

    int command = 0;
    if (rpm == 0)
      command = 10;
    else if (rpm > 0)
      command = 26;
    else if (rpm < 0)
      command = 58;

    rpm = std::abs(rpm);
    // Trigger = 1 (normal start)
    const int mode = 16; // Continuous speed control
    const int position = 0;
    const int acc_rate = 1000;
    const int dec_rate = 2500;
    const int torque = 1000; // 50.0%
    const int trigger = 1;
    std::cout << "rpm" << rpm;
    auto write32 = [&](int start_reg, int value) -> bool
    {
      uint16_t regs[2];
      regs[1] = value & 0xFFFF;         // LSW
      regs[0] = (value >> 16) & 0xFFFF; // MSW
      int rc = modbus_write_registers(ctx_, start_reg, 2, regs);
      if (rc == -1)
      {
        std::cerr << "[Motor " << motor_id << "] Failed to write to reg " << std::hex << start_reg
                  << ": " << modbus_strerror(errno) << "\n";
        return false;
      }
      return true;
    };

    bool ok =
        write32(90, 16) &&
        write32(92, position) &&
        write32(94, 1000) &&
        write32(96, acc_rate) &&
        write32(98, dec_rate) &&
        write32(100, torque) &&
        write32(102, trigger);
    return ok
               ? 0
               : -1;
  }

  int BlvrComunicator::readRegister(int motor_id, int addr, int *value)
  {
    if (!ctx_)
      return -1;
    modbus_set_slave(ctx_, motor_id);

    uint16_t data[2];
    int rc = modbus_read_registers(ctx_, addr, 2, data);
    if (rc != 2)
      return -1;

    *value = (data[0] << 16) | data[1];
    return 0;
  }

  int BlvrComunicator::writeRegister(int motor_id, int addr, int value)
  {
    if (!ctx_)
      return -1;
    modbus_set_slave(ctx_, motor_id);

    uint16_t data[2];
    data[0] = (value >> 16) & 0xFFFF;         // Lower 16 bits first (LSW)
    data[1] = value & 0xFFFF; // Upper 16 bits next (MSW)

    int rc = modbus_write_registers(ctx_, addr, 2, data);
    if (rc == -1)
    {
      std::cerr << "Write failed to slave " << motor_id << " at addr " << std::hex << addr
                << ": " << modbus_strerror(errno) << "\n";
    }
    usleep(10000);
    return rc;
  }

  int BlvrComunicator::writeSingleRegister(int motor_id, int addr, int value)
  {
    modbus_set_slave(ctx_, motor_id);
    usleep(1000);

    int rc = modbus_write_register(ctx_, addr, value);
    if (rc == -1)
    {
      std::cerr << "Write failed to slave " << motor_id << " at addr " << std::hex << addr
                << ": " << modbus_strerror(errno) << "\n";
    }
    return rc;
  }

} // namespace blvr_diffbot_hardware
