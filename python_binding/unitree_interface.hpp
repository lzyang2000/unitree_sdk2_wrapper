#pragma once

#include <memory>
#include <mutex>
#include <shared_mutex>
#include <array>
#include <cmath>
#include <string>

// DDS
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// IDL - HG (Humanoid/Go1)
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

// IDL - GO2 (Go2)
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>

// IDL - Hand control
#include <unitree/idl/hg/HandCmd_.hpp>
#include <unitree/idl/hg/HandState_.hpp>

// IDL - Dex1-1 gripper (parallel-jaw, 1 DOF, go2-namespace MotorCmds_/MotorStates_)
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>

#include "unitree/common/thread/thread.hpp"

using namespace unitree::common;
using namespace unitree::robot;

// Topic definitions
static const std::string HG_CMD_TOPIC = "rt/lowcmd";
static const std::string HG_STATE_TOPIC = "rt/lowstate";
static const std::string GO2_CMD_TOPIC = "rt/lowcmd";
static const std::string GO2_STATE_TOPIC = "rt/lowstate";
static const std::string TOPIC_JOYSTICK = "rt/wirelesscontroller";

// Hand topic definitions
static const std::string LEFT_HAND_CMD_TOPIC = "rt/dex3/left/cmd";
static const std::string LEFT_HAND_STATE_TOPIC = "rt/dex3/left/state";
static const std::string RIGHT_HAND_CMD_TOPIC = "rt/dex3/right/cmd";
static const std::string RIGHT_HAND_STATE_TOPIC = "rt/dex3/right/state";

// Dex1-1 gripper topics (1-DOF parallel jaw, separate from Dex3-1 hand topics)
static const std::string LEFT_DEX1_CMD_TOPIC = "rt/dex1/left/cmd";
static const std::string LEFT_DEX1_STATE_TOPIC = "rt/dex1/left/state";
static const std::string RIGHT_DEX1_CMD_TOPIC = "rt/dex1/right/cmd";
static const std::string RIGHT_DEX1_STATE_TOPIC = "rt/dex1/right/state";

// Robot configurations
enum class RobotType {
    G1 = 0,      // G1 humanoid (29 motors)
    H1 = 1,      // H1 humanoid (19 motors) - uses GO2 messages
    H1_2 = 2,    // H1-2 humanoid (29 motors)
    CUSTOM = 99  // Custom robot with specified motor count
};

enum class MessageType {
    HG = 0,   // Humanoid/Go1 message format
    GO2 = 1   // Go2 message format
};

// Hand configurations
enum class HandType {
    LEFT_HAND = 0,
    RIGHT_HAND = 1
};

// Dex1-1 gripper side selector (1-DOF parallel-jaw, distinct from HandType
// which addresses the 7-DOF Dex3-1 hand).
enum class Dex1Type {
    LEFT = 0,
    RIGHT = 1
};

// Constants for Dex3-1 hands
static const int DEX3_NUM_MOTORS = 7;
static const int DEX3_NUM_PRESS_SENSORS = 9;

// Robot configuration structure
struct RobotConfig {
    RobotType robot_type;
    MessageType message_type;
    int num_motors;
    std::string name;
    
    RobotConfig(RobotType rt, MessageType mt, int nm, const std::string& n)
        : robot_type(rt), message_type(mt), num_motors(nm), name(n) {}
};

// Predefined robot configurations
namespace RobotConfigs {
    static const RobotConfig G1_HG(RobotType::G1, MessageType::HG, 29, "G1-HG");
    static const RobotConfig H1_GO2(RobotType::H1, MessageType::GO2, 19, "H1-GO2");
    static const RobotConfig H1_2_HG(RobotType::H1_2, MessageType::HG, 29, "H1-2-HG");
}

template <typename T>
class DataBuffer {
 public:
  void SetData(const T &newData) {
    std::unique_lock<std::shared_mutex> lock(mutex);
    data = std::make_shared<T>(newData);
  }

  std::shared_ptr<const T> GetData() {
    std::shared_lock<std::shared_mutex> lock(mutex);
    return data ? data : nullptr;
  }

  void Clear() {
    std::unique_lock<std::shared_mutex> lock(mutex);
    data = nullptr;
  }

 private:
  std::shared_ptr<T> data;
  std::shared_mutex mutex;
};

// Python-exposed data structures
struct PyImuState {
  std::array<float, 3> rpy = {};
  std::array<float, 3> omega = {};
  std::array<float, 4> quat = {};
  std::array<float, 3> accel = {};
};

struct PyMotorState {
  std::vector<float> q;
  std::vector<float> dq;
  std::vector<float> tau_est;
  std::vector<int> temperature;
  std::vector<float> voltage;
  
  PyMotorState(int num_motors) : q(num_motors), dq(num_motors), tau_est(num_motors), 
                                 temperature(num_motors), voltage(num_motors) {}
};

struct PyMotorCommand {
  std::vector<float> q_target;
  std::vector<float> dq_target;
  std::vector<float> kp;
  std::vector<float> kd;
  std::vector<float> tau_ff;
  
  PyMotorCommand(int num_motors) : q_target(num_motors), dq_target(num_motors), 
                                   kp(num_motors), kd(num_motors), tau_ff(num_motors) {}
};

struct PyWirelessController {
  float lx;
  float ly;
  float rx;
  float ry;
  uint16_t keys;
};

struct PyLowState {
  PyImuState imu;
  PyMotorState motor;
  uint8_t mode_machine = 0;
  
  PyLowState(int num_motors) : motor(num_motors) {}
};

// Hand-specific Python data structures
struct PyHandMotorState {
  std::array<float, DEX3_NUM_MOTORS> q = {};
  std::array<float, DEX3_NUM_MOTORS> dq = {};
  std::array<float, DEX3_NUM_MOTORS> tau_est = {};
  std::array<std::array<float, 2>, DEX3_NUM_MOTORS> temperature = {};
  std::array<float, DEX3_NUM_MOTORS> voltage = {};
};

struct PyHandMotorCommand {
  std::array<float, DEX3_NUM_MOTORS> q_target = {};
  std::array<float, DEX3_NUM_MOTORS> dq_target = {};
  std::array<float, DEX3_NUM_MOTORS> kp = {};
  std::array<float, DEX3_NUM_MOTORS> kd = {};
  std::array<float, DEX3_NUM_MOTORS> tau_ff = {};
};

struct PyHandPressSensorState {
  std::array<std::array<float, 12>, DEX3_NUM_PRESS_SENSORS> pressure = {};
  std::array<std::array<float, 12>, DEX3_NUM_PRESS_SENSORS> temperature = {};
  std::array<uint8_t, DEX3_NUM_PRESS_SENSORS> lost = {};
  std::array<std::array<uint32_t, 4>, DEX3_NUM_PRESS_SENSORS> reserve = {};
};

struct PyHandImuState {
  std::array<float, 4> quaternion = {};
  std::array<float, 3> gyroscope = {};
  std::array<float, 3> accelerometer = {};
  std::array<float, 3> rpy = {};
  float temperature = 0.0f;
};

struct PyHandState {
  PyHandMotorState motor;
  PyHandPressSensorState press_sensor;
  PyHandImuState imu;
  float power_v = 0.0f;
  float power_a = 0.0f;
  float system_v = 0.0f;
  float device_v = 0.0f;
  std::array<uint32_t, 2> error = {};
  std::array<uint32_t, 2> reserve = {};
};

// Dex1-1 single-motor state (parallel-jaw, 1 DOF). Mirrors the per-motor
// fields the dex1_1_service emits in MotorStates_.states()[0].
struct PyDex1State {
  float q = 0.0f;
  float dq = 0.0f;
  float tau_est = 0.0f;
  uint8_t temperature = 0;
};

// Dex1-1 single-motor command. mode=1 selects position control in the
// service (matches test_gripper.cpp in dex1_1_service).
struct PyDex1Command {
  uint8_t mode = 1;
  float q_target = 0.0f;
  float dq_target = 0.0f;
  float kp = 0.0f;
  float kd = 0.0f;
  float tau_ff = 0.0f;
};

enum class PyControlMode {
  PR = 0,  // Pitch/Roll mode
  AB = 1   // A/B mode
};

// Internal data structures
struct ImuState {
  std::array<float, 3> rpy = {};
  std::array<float, 3> omega = {};
  std::array<float, 4> quat = {};
  std::array<float, 3> accel = {};
};

struct MotorCommand {
  std::vector<float> q_target;
  std::vector<float> dq_target;
  std::vector<float> kp;
  std::vector<float> kd;
  std::vector<float> tau_ff;
  
  MotorCommand(int num_motors) : q_target(num_motors), dq_target(num_motors), 
                                 kp(num_motors), kd(num_motors), tau_ff(num_motors) {}
};

struct MotorState {
  std::vector<float> q;
  std::vector<float> dq;
  std::vector<float> tau_est;
  std::vector<int> temperature;
  std::vector<float> voltage;
  
  MotorState(int num_motors) : q(num_motors), dq(num_motors), tau_est(num_motors), 
                               temperature(num_motors), voltage(num_motors) {}
};

// CRC function
inline uint32_t Crc32Core(uint32_t *ptr, uint32_t len) {
  uint32_t xbit = 0;
  uint32_t data = 0;
  uint32_t CRC32 = 0xFFFFFFFF;
  const uint32_t dwPolynomial = 0x04c11db7;
  for (uint32_t i = 0; i < len; i++) {
    xbit = 1 << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++) {
      if (CRC32 & 0x80000000) {
        CRC32 <<= 1;
        CRC32 ^= dwPolynomial;
      } else
        CRC32 <<= 1;
      if (data & xbit) CRC32 ^= dwPolynomial;

      xbit >>= 1;
    }
  }
  return CRC32;
}

class UnitreeInterface {
 private:
  RobotConfig config_;
  PyControlMode mode_;
  uint8_t mode_machine_;
  
  DataBuffer<MotorState> motor_state_buffer_;
  DataBuffer<MotorCommand> motor_command_buffer_;
  DataBuffer<ImuState> imu_state_buffer_;
  DataBuffer<PyWirelessController> wireless_controller_buffer_;

  // DDS components - using void pointers for type flexibility
  std::shared_ptr<void> lowcmd_publisher_;
  std::shared_ptr<void> lowstate_subscriber_;
  std::shared_ptr<void> wireless_subscriber_;
  
  ThreadPtr command_writer_ptr_;
  std::mutex wireless_mutex_;

  // Default gains
  std::vector<float> default_kp_;
  std::vector<float> default_kd_;

  void InitDefaultGains();
  void LowStateHandler(const void *message);
  void WirelessControllerHandler(const void *message);
  void LowCommandWriter();
  
  // Convert internal structures to Python structures
  PyLowState ConvertToPyLowState();
  MotorCommand ConvertFromPyMotorCommand(const PyMotorCommand& py_cmd);
  
  // Template helper functions for different message types
  template<typename LowStateType>
  void ProcessLowState(const LowStateType& low_state);
  
  // Specific implementations for different message types
  void ProcessLowState(const unitree_hg::msg::dds_::LowState_& low_state);
  void ProcessLowState(const unitree_go::msg::dds_::LowState_& low_state);
  
  template<typename LowCmdType>
  void WriteLowCommand(LowCmdType& low_cmd);

  // DDS initialization
  void InitializeDDS(const std::string& networkInterface);

 public:
  // Constructors
  UnitreeInterface(const std::string& networkInterface, RobotType robot_type, MessageType message_type);
  UnitreeInterface(const std::string& networkInterface, const RobotConfig& config);
  UnitreeInterface(const std::string& networkInterface, RobotType robot_type, MessageType message_type, int num_motors);
  
  ~UnitreeInterface();
  
  // Python interface methods
  PyLowState ReadLowState();
  PyWirelessController ReadWirelessController();
  void WriteLowCommand(const PyMotorCommand& command);
  void SetControlMode(PyControlMode mode);
  PyControlMode GetControlMode() const;
  
  // Utility methods
  PyMotorCommand CreateZeroCommand();
  std::vector<float> GetDefaultKp() const;
  std::vector<float> GetDefaultKd() const;
  
  // Configuration methods
  RobotConfig GetConfig() const { return config_; }
  int GetNumMotors() const { return config_.num_motors; }
  std::string GetRobotName() const { return config_.name; }

  // Helper functions
  static int GetDefaultMotorCount(RobotType robot_type);
  static std::string GetRobotName(RobotType robot_type, MessageType message_type);
  
  // Static factory methods
  static std::shared_ptr<UnitreeInterface> CreateG1(const std::string& networkInterface, MessageType message_type = MessageType::HG);
  static std::shared_ptr<UnitreeInterface> CreateH1(const std::string& networkInterface, MessageType message_type = MessageType::GO2);
  static std::shared_ptr<UnitreeInterface> CreateH1_2(const std::string& networkInterface, MessageType message_type = MessageType::HG);
  static std::shared_ptr<UnitreeInterface> CreateCustom(const std::string& networkInterface, int num_motors, MessageType message_type = MessageType::HG);
};

// Hand Interface Class
class HandInterface {
 private:
  HandType hand_type_;
  std::string hand_name_;
  
  DataBuffer<PyHandMotorState> motor_state_buffer_;
  DataBuffer<PyHandMotorCommand> motor_command_buffer_;
  DataBuffer<PyHandPressSensorState> press_sensor_buffer_;
  DataBuffer<PyHandImuState> imu_buffer_;

  // DDS components
  std::shared_ptr<ChannelPublisher<unitree_hg::msg::dds_::HandCmd_>> hand_cmd_publisher_;
  std::shared_ptr<ChannelSubscriber<unitree_hg::msg::dds_::HandState_>> hand_state_subscriber_;
  
  ThreadPtr command_writer_ptr_;

  // Default gains
  std::array<float, DEX3_NUM_MOTORS> default_kp_;
  std::array<float, DEX3_NUM_MOTORS> default_kd_;

  void InitDefaultGains();
  void HandStateHandler(const void *message);
  void HandCommandWriter();
  
  // Convert internal structures to Python structures
  PyHandState ConvertToPyHandState();
  
  // DDS initialization (with re_init support)
  void InitializeDDS(const std::string& networkInterface, bool re_init);

  // Get joint limits for current hand
  const std::array<float, DEX3_NUM_MOTORS>& GetMaxLimits() const;
  const std::array<float, DEX3_NUM_MOTORS>& GetMinLimits() const;

 public:
  // Constructors
  explicit HandInterface(const std::string& networkInterface, HandType hand_type, bool re_init = true);
  ~HandInterface();
  
  // Python interface methods
  PyHandState ReadHandState();
  void WriteHandCommand(const PyHandMotorCommand& command);
  
  // Utility methods
  PyHandMotorCommand CreateZeroCommand();
  PyHandMotorCommand CreateDefaultCommand();
  std::array<float, DEX3_NUM_MOTORS> GetDefaultKp() const;
  std::array<float, DEX3_NUM_MOTORS> GetDefaultKd() const;
  std::array<float, DEX3_NUM_MOTORS> GetMaxLimitsArray();
  std::array<float, DEX3_NUM_MOTORS> GetMinLimitsArray();
  
  // Configuration methods
  HandType GetHandType() const { return hand_type_; }
  std::string GetHandName() const { return hand_name_; }
  
  // Joint manipulation utilities
  void ClampJointAngles(std::array<float, DEX3_NUM_MOTORS>& joint_angles) const;
  std::array<float, DEX3_NUM_MOTORS> NormalizeJointAngles(const std::array<float, DEX3_NUM_MOTORS>& joint_angles) const;
  
  // Static factory methods
  static std::shared_ptr<HandInterface> CreateLeftHand(const std::string& networkInterface, bool re_init = true);
  static std::shared_ptr<HandInterface> CreateRightHand(const std::string& networkInterface, bool re_init = true);
};

// Dex1-1 gripper interface (1-DOF parallel jaw, MotorCmds_/MotorStates_
// over rt/dex1/{left,right}/{cmd,state}). Mirrors HandInterface's threading
// model: a recurrent writer thread re-publishes the latest buffered command,
// while incoming state is delivered via a callback handler into a buffer.
class Dex1Interface {
 private:
  Dex1Type side_;
  std::string side_name_;

  DataBuffer<PyDex1State> state_buffer_;
  DataBuffer<PyDex1Command> command_buffer_;

  std::shared_ptr<ChannelPublisher<unitree_go::msg::dds_::MotorCmds_>> cmd_publisher_;
  std::shared_ptr<ChannelSubscriber<unitree_go::msg::dds_::MotorStates_>> state_subscriber_;

  ThreadPtr command_writer_ptr_;

  float default_kp_;
  float default_kd_;

  void InitDefaultGains();
  void StateHandler(const void *message);
  void CommandWriter();

  void InitializeDDS(const std::string& networkInterface, bool re_init);

 public:
  explicit Dex1Interface(const std::string& networkInterface, Dex1Type side, bool re_init = true);
  ~Dex1Interface();

  // Python interface
  PyDex1State ReadState();
  void WriteCommand(const PyDex1Command& command);

  // Utility
  PyDex1Command CreateZeroCommand();
  float GetDefaultKp() const { return default_kp_; }
  float GetDefaultKd() const { return default_kd_; }
  Dex1Type GetSide() const { return side_; }
  std::string GetSideName() const { return side_name_; }

  static std::shared_ptr<Dex1Interface> CreateLeft(const std::string& networkInterface, bool re_init = true);
  static std::shared_ptr<Dex1Interface> CreateRight(const std::string& networkInterface, bool re_init = true);
};
