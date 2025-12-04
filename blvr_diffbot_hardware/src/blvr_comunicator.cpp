#include "blvr_diffbot_hardware/blvr_comunicator.h"

#include <iostream>
#include <unistd.h>
#include <cerrno>

namespace blvr_diffbot_hardware
{

std::mutex modbus_mutex_;
static int last_slave_id = -1;

BlvrComunicator::BlvrComunicator() : ctx_(nullptr) {}

BlvrComunicator::~BlvrComunicator()
{
  if (ctx_) {
    modbus_close(ctx_);
    modbus_free(ctx_);
    ctx_ = nullptr;
  }
}

BlvrComunicator::return_type BlvrComunicator::openDevice(
    const std::string &device, int baud, char parity, int data_bit, int stop_bit)
{
  ctx_ = modbus_new_rtu(device.c_str(), baud, parity, data_bit, stop_bit);
  if (!ctx_) {
    std::cerr << "Unable to allocate libmodbus context\n";
    return return_type::ERROR;
  }

  // Tight timeouts to avoid blocking the control loop
  modbus_set_response_timeout(ctx_, 0, 20000); // 20 ms
  modbus_set_byte_timeout(ctx_,     0, 2000);  // 2 ms/byte

  if (modbus_connect(ctx_) == -1) {
    std::cerr << "Connection failed: " << modbus_strerror(errno) << std::endl;
    modbus_free(ctx_);
    ctx_ = nullptr;
    return return_type::ERROR;
  }

  last_slave_id = -1;
  return return_type::SUCCESS;
}

void BlvrComunicator::closeDevice()
{
  std::lock_guard<std::mutex> lock(modbus_mutex_);
  if (ctx_) {
    modbus_close(ctx_);
    modbus_free(ctx_);
    ctx_ = nullptr;
    last_slave_id = -1;
  }
}

int BlvrComunicator::setExcitation(int motor_id)
{
  usleep(2000);
  // Drive-specific: 0x007C ← 1 (32-bit, MSW first)
  return writeRegister(motor_id, 0x007C, 1);
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

int BlvrComunicator::readStepRpmTorque(int motor_id, int* step, int* rpm, int* torque)
{
  if (!ctx_) return -1;
  std::lock_guard<std::mutex> lock(modbus_mutex_);

  if (last_slave_id != motor_id) {
    if (modbus_set_slave(ctx_, motor_id) == -1) return -1;
    last_slave_id = motor_id;
  }

  // Read: CC..D7 (12 regs)
  constexpr int start = 0x00CC;
  constexpr int count = 12;
  uint16_t reg[count] = {0};

  int rc = modbus_read_registers(ctx_, start, count, reg);
  if (rc != count) return -1;

  // MSW first, then LSW
  auto join32 = [](uint16_t msw, uint16_t lsw) -> int {
    return static_cast<int>((static_cast<uint32_t>(msw) << 16) | static_cast<uint32_t>(lsw));
  };

  *step   = join32(reg[0],  reg[1]);   // CC, CD
  *rpm    = join32(reg[2],  reg[3]);   // CE, CF
  *torque = join32(reg[10], reg[11]);  // D6, D7

  return 0;
}

int BlvrComunicator::broadcastTrigger(int start_reg, int pulse_us)
{
  if (!ctx_) return -1;
  if (pulse_us < 1000) pulse_us = 1000;

  std::lock_guard<std::mutex> lock(modbus_mutex_);

  // Broadcast address
  if (modbus_set_slave(ctx_, 0) == -1) return -1;

  // ON (value = 1 as 32-bit) — MSW first then LSW
  uint16_t regs_on[2];
  regs_on[0] = 0x0000; // MSW
  regs_on[1] = 0x0001; // LSW
  if (modbus_write_registers(ctx_, start_reg, 2, regs_on) != 2) return -1;

  // Optional OFF (0) if a falling edge is needed:
  // usleep(pulse_us);
  // uint16_t regs_off[2] = {0x0000, 0x0000};
  // if (modbus_write_registers(ctx_, start_reg, 2, regs_off) != 2) return -1;

  last_slave_id = -1; // force reset on next unicast
  return 0;
}

int BlvrComunicator::immediateStop()
{
  if (!ctx_) return -1;

  std::lock_guard<std::mutex> lock(modbus_mutex_);

  // 1. Set to broadcast address
  modbus_set_slave(ctx_, 0);

  // 2. Mode = 32 (immediate stop) at reg 90 (2 words, MSW/LSW)
  uint16_t regs_mode[2];
  regs_mode[0] = 0x0000;  // MSW
  regs_mode[1] = 32;      // LSW (value 32)
  modbus_write_registers(ctx_, 90, 2, regs_mode);

  // 3. Broadcast trigger at reg 102
  uint16_t regs_on[2]  = {0x0000, 0x0001};
  uint16_t regs_off[2] = {0x0000, 0x0000};

  modbus_write_registers(ctx_, 102, 2, regs_on);
  usleep(10000); // 10 ms pulse
  modbus_write_registers(ctx_, 102, 2, regs_off);

  last_slave_id = -1;

  std::cerr << "[BlvrComunicator] Immediate STOP broadcast issued\n";
  return 0;
}


int BlvrComunicator::writeSpeedCommand(int motor_id, int rpm)
{
  if (!ctx_) return -1;

  std::lock_guard<std::mutex> lock(modbus_mutex_);

  if (last_slave_id != motor_id) {
    if (modbus_set_slave(ctx_, motor_id) == -1) return -1;
    last_slave_id = motor_id;
  }

  // Parameter block (drive-specific)
  const int mode     = 16;    // speed mode
  const int position = 0;
  const int acc_rate = 450;
  const int dec_rate = 220;
  // const int acc_rate = 450;
  // const int dec_rate = 220;
  const int torque   = 800;

  auto write32 = [&](int start_reg, int value) -> bool {
    // MSW first, then LSW
    uint16_t regs[2];
    regs[0] = static_cast<uint16_t>((value >> 16) & 0xFFFF); // MSW
    regs[1] = static_cast<uint16_t>( value        & 0xFFFF); // LSW
    int rc = modbus_write_registers(ctx_, start_reg, 2, regs);
    return rc == 2;
  };

  bool ok =
      write32(90,  mode)     &&
      write32(92,  position) &&
      write32(94,  rpm)      &&
      write32(96,  acc_rate) &&
      write32(98,  dec_rate) &&
      write32(100, torque);

  return ok ? 0 : -1;
}

int BlvrComunicator::readRegister(int motor_id, int addr, int *value)
{
  if (!ctx_) return -1;

  std::lock_guard<std::mutex> lock(modbus_mutex_);

  if (last_slave_id != motor_id) {
    if (modbus_set_slave(ctx_, motor_id) == -1) return -1;
    last_slave_id = motor_id;
  }

  uint16_t data[2] = {0,0};
  int rc = modbus_read_registers(ctx_, addr, 2, data);
  if (rc != 2) return -1;

  // MSW: data[0], LSW: data[1]
  *value = (static_cast<int>(data[0]) << 16) | static_cast<int>(data[1]);
  return 0;
}

int BlvrComunicator::writeRegister(int motor_id, int addr, int value)
{
  if (!ctx_) return -1;

  std::lock_guard<std::mutex> lock(modbus_mutex_);

  if (last_slave_id != motor_id) {
    if (modbus_set_slave(ctx_, motor_id) == -1) return -1;
    last_slave_id = motor_id;
  }

  // MSW first, then LSW
  uint16_t data[2];
  data[0] = static_cast<uint16_t>((value >> 16) & 0xFFFF); // MSW
  data[1] = static_cast<uint16_t>( value        & 0xFFFF); // LSW

  int rc = modbus_write_registers(ctx_, addr, 2, data);
  return (rc == 2) ? rc : -1;
}

int BlvrComunicator::writeSingleRegister(int motor_id, int addr, int value)
{
  if (!ctx_) return -1;

  std::lock_guard<std::mutex> lock(modbus_mutex_);

  if (last_slave_id != motor_id) {
    if (modbus_set_slave(ctx_, motor_id) == -1) return -1;
    last_slave_id = motor_id;
  }

  return modbus_write_register(ctx_, addr, static_cast<uint16_t>(value));
}

} // namespace blvr_diffbot_hardware
