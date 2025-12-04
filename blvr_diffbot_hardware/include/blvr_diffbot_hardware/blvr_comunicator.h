#pragma once

#include <string>
#include <cstdint>
#include <mutex>

#include <modbus/modbus.h>

namespace blvr_diffbot_hardware
{

class BlvrComunicator
{
public:
  enum class return_type { SUCCESS = 0, ERROR = 1 };

  BlvrComunicator();
  ~BlvrComunicator();

  // Device
  return_type openDevice(const std::string &device, int baud, char parity, int data_bit, int stop_bit);
  void closeDevice();

  // 32-bit registers (pair of holding registers)
  // WORD ORDER: MSW first, then LSW (MSW at addr, LSW at addr+1)
  int readRegister(int motor_id, int addr, int *value);
  int writeRegister(int motor_id, int addr, int value);
  int writeSingleRegister(int motor_id, int addr, int value);
  int immediateStop();

  // High-level helpers
  int setExcitation(int motor_id);
  int readStep(int motor_id, int *step);
  int readRpm(int motor_id, int *rpm);
  int readTorque(int motor_id, int *torque);

  // Optimized block read: CC..D7 (12 regs) → step/rpm/torque (32-bit)
  int readStepRpmTorque(int motor_id, int* step, int* rpm, int* torque);

  // Broadcast a 32-bit trigger pulse at `start_reg` (MSW first, then LSW)
  int broadcastTrigger(int start_reg, int pulse_us);

  // Stage a speed command (writes the parameter block for speed mode)
  int writeSpeedCommand(int motor_id, int rpm);

private:
  modbus_t *ctx_;
};

// Global shared mutex for libmodbus thread-safety inside this process
extern std::mutex modbus_mutex_;

} // namespace blvr_diffbot_hardware
