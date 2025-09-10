#include "blvr_diffbot_hardware/blvr_comunicator.h"

#include <iostream>
#include <unistd.h>
#include <mutex>
#include <map>

namespace blvr_diffbot_hardware
{

  std::mutex modbus_mutex_;
  static int last_slave_id = -1;  // To avoid unnecessary modbus_set_slave calls

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
    if (!ctx_)
    {
      std::cerr << "Unable to allocate libmodbus context" << std::endl;
      exit(1);
    }

    modbus_set_response_timeout(ctx_, 1, 20000);
    modbus_set_byte_timeout(ctx_, 0, 2000);

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
    std::lock_guard<std::mutex> lock(modbus_mutex_);
    if (ctx_)
    {
      modbus_close(ctx_);
      modbus_free(ctx_);
      ctx_ = nullptr;
      last_slave_id = -1;
    }
  }

  int BlvrComunicator::setExcitation(int motor_id)
  {
    return writeRegister(motor_id, 0x007C, 1);
  }

  int BlvrComunicator::readStep(int motor_id, int *step)
  {
    // usleep(3000);
    return readRegister(motor_id, 0x00CC, step);
  }

  int BlvrComunicator::readRpm(int motor_id, int *rpm)
  {
    // usleep(3000);
    return readRegister(motor_id, 0x00CE, rpm);
  }

  int BlvrComunicator::readTorque(int motor_id, int *torque)
  {
    // usleep(3000);
    return readRegister(motor_id, 0x00D6, torque);
  }


  // BlvrComunicator.cpp
int BlvrComunicator::readStepRpmTorque(int motor_id, int* step, int* rpm, int* torque)
{
  if (!ctx_) return -1;
  std::lock_guard<std::mutex> lock(modbus_mutex_);

  if (last_slave_id != motor_id) {
    modbus_set_slave(ctx_, motor_id);
    last_slave_id = motor_id;
  }

  // Read contiguous block CC..D7 (12 regs)
  constexpr int start = 0x00CC;
  constexpr int count = 12; // CC,CD,CE,CF,D0,D1,D2,D3,D4,D5,D6,D7
  uint16_t reg[count] = {0};

  int rc = modbus_read_registers(ctx_, start, count, reg);
  if (rc != count) return -1;

  // Keep YOUR original combine order: (MSW << 16) | LSW
  auto join32 = [](uint16_t msw, uint16_t lsw) -> int {
    return static_cast<int>((static_cast<uint32_t>(msw) << 16) | static_cast<uint32_t>(lsw));
  };

  // Offsets: CC->0, CD->1, CE->2, CF->3, D6->10, D7->11
  *step   = join32(reg[0],  reg[1]);   // STEP: CC,CD
  *rpm    = join32(reg[2],  reg[3]);   // RPM : CE,CF
  *torque = join32(reg[10], reg[11]);  // TORQ: D6,D7

  return 0;
}


  int BlvrComunicator::broadcastTrigger(int start_reg, int pulse_us)
{
  if (!ctx_) return -1;
  if (pulse_us < 1000) pulse_us = 1000;

  std::lock_guard<std::mutex> lock(modbus_mutex_);

  // set to broadcast
  modbus_set_slave(ctx_, 0);

  // ON (value = 1 as 32-bit)
  uint16_t regs_on[2];
  regs_on[0] = 0x0000; // MSW
  regs_on[1] = 0x0001; // LSW
  modbus_write_registers(ctx_, start_reg, 2, regs_on);

  // usleep(pulse_us);

  // // OFF (value = 0 as 32-bit)
  // uint16_t regs_off[2];
  // regs_off[0] = 0x0000; // MSW
  // regs_off[1] = 0x0000; // LSW
  // modbus_write_registers(ctx_, start_reg, 2, regs_off);

  // reset last_slave_id so next per-motor call re-sets
  last_slave_id = -1;

  return 0;
}


  int BlvrComunicator::writeSpeedCommand(int motor_id, int rpm)
  {
    if (!ctx_)
      return -1;

    std::lock_guard<std::mutex> lock(modbus_mutex_);

    if (last_slave_id != motor_id)
    {
      modbus_set_slave(ctx_, motor_id);
      last_slave_id = motor_id;
    }

    const int mode = 16;
    const int position = 0;
    const int acc_rate = 1000;
    const int dec_rate = 800;
    const int torque = 1000;
    const int trigger = 1;

    auto write32 = [&](int start_reg, int value) -> bool {
      uint16_t regs[2];
      regs[1] = value & 0xFFFF;
      regs[0] = (value >> 16) & 0xFFFF;
      for (int attempt = 0; attempt < 3; ++attempt)
      {
        int rc = modbus_write_registers(ctx_, start_reg, 2, regs);
        if (rc == 2)
        {
          // usleep(20000);
          return true;
        }
        // usleep(8000);
      }
      return false;
    };

    bool ok =
        write32(90, mode) &&
        write32(92, position) &&
        write32(94, rpm) &&
        write32(96, acc_rate) &&
        write32(98, dec_rate) &&
        write32(100, torque) ;
        // write32(102, trigger);

    return ok ? 0 : -1;
  }


  int BlvrComunicator::readRegister(int motor_id, int addr, int *value)
  {
    if (!ctx_)
      return -1;

    std::lock_guard<std::mutex> lock(modbus_mutex_);

    if (last_slave_id != motor_id)
    {
      modbus_set_slave(ctx_, motor_id);
      last_slave_id = motor_id;
    }

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

    std::lock_guard<std::mutex> lock(modbus_mutex_);

    if (last_slave_id != motor_id)
    {
      modbus_set_slave(ctx_, motor_id);
      last_slave_id = motor_id;
    }

    uint16_t data[2];
    data[0] = (value >> 16) & 0xFFFF;
    data[1] = value & 0xFFFF;

    for (int attempt = 0; attempt < 3; ++attempt)
    {
      int rc = modbus_write_registers(ctx_, addr, 2, data);
      if (rc == 2)
      {
        usleep(10000);
        return rc;
      }
      usleep(8000);
    }
    return -1;
  }

  int BlvrComunicator::writeSingleRegister(int motor_id, int addr, int value)
  {
    std::lock_guard<std::mutex> lock(modbus_mutex_);

    if (last_slave_id != motor_id)
    {
      modbus_set_slave(ctx_, motor_id);
      last_slave_id = motor_id;
    }

    usleep(1000);
    int rc = modbus_write_register(ctx_, addr, value);
    return rc;
  }

} // namespace blvr_diffbot_hardware