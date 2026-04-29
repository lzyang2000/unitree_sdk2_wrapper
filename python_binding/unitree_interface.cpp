#include "unitree_interface.hpp"
#include <iostream>
#include <unistd.h>
#include <iomanip>
#include <chrono>
#include <thread>

// Constructor implementations
UnitreeInterface::UnitreeInterface(const std::string& networkInterface, RobotType robot_type, MessageType message_type)
    : config_(robot_type, message_type, GetDefaultMotorCount(robot_type), GetRobotName(robot_type, message_type)),
      mode_(PyControlMode::PR), mode_machine_(0) {
    
    InitDefaultGains();
    InitializeDDS(networkInterface);
}

UnitreeInterface::UnitreeInterface(const std::string& networkInterface, const RobotConfig& config)
    : config_(config), mode_(PyControlMode::PR), mode_machine_(0) {
    
    InitDefaultGains();
    InitializeDDS(networkInterface);
}

UnitreeInterface::UnitreeInterface(const std::string& networkInterface, RobotType robot_type, MessageType message_type, int num_motors)
    : config_(robot_type, message_type, num_motors, GetRobotName(robot_type, message_type)),
      mode_(PyControlMode::PR), mode_machine_(0) {
    
    InitDefaultGains();
    InitializeDDS(networkInterface);
}

UnitreeInterface::~UnitreeInterface() {
    if (command_writer_ptr_) {
        command_writer_ptr_.reset();
    }
    lowcmd_publisher_.reset();
    lowstate_subscriber_.reset();
    wireless_subscriber_.reset();
}

// Helper functions for robot configuration
int UnitreeInterface::GetDefaultMotorCount(RobotType robot_type) {
    switch (robot_type) {
        case RobotType::G1: return 29;
        case RobotType::H1: return 19;
        case RobotType::H1_2: return 29;
        default: return 12;
    }
}

std::string UnitreeInterface::GetRobotName(RobotType robot_type, MessageType message_type) {
    std::string robot_name;
    switch (robot_type) {
        case RobotType::G1: robot_name = "G1"; break;
        case RobotType::H1: robot_name = "H1"; break;
        case RobotType::H1_2: robot_name = "H1-2"; break;
        default: robot_name = "CUSTOM"; break;
    }
    
    std::string msg_type = (message_type == MessageType::HG) ? "HG" : "GO2";
    return robot_name + "-" + msg_type;
}

void UnitreeInterface::InitDefaultGains() {
    int num_motors = config_.num_motors;
    default_kp_.resize(num_motors);
    default_kd_.resize(num_motors);
    
    // Set default gains based on robot type
    switch (config_.robot_type) {
        case RobotType::G1:
        case RobotType::H1_2:
            // G1/H1-2 gains (29 motors)
            for (int i = 0; i < 12; ++i) {  // Legs
                default_kp_[i] = 60.0f;
                default_kd_[i] = 1.0f;
            }
            for (int i = 12; i < 15; ++i) {  // Waist
                default_kp_[i] = 60.0f;
                default_kd_[i] = 1.0f;
            }
            for (int i = 15; i < num_motors; ++i) {  // Arms
                default_kp_[i] = 40.0f;
                default_kd_[i] = 1.0f;
            }
            break;
            
        case RobotType::H1:
            // H1 gains (19 motors)
            for (int i = 0; i < 12; ++i) {  // Legs
                default_kp_[i] = 60.0f;
                default_kd_[i] = 1.0f;
            }
            for (int i = 12; i < 15; ++i) {  // Waist
                default_kp_[i] = 60.0f;
                default_kd_[i] = 1.0f;
            }
            for (int i = 15; i < num_motors; ++i) {  // Arms (4 motors)
                default_kp_[i] = 40.0f;
                default_kd_[i] = 1.0f;
            }
            break;
            
        default:
            // Custom robot
            for (int i = 0; i < num_motors; ++i) {
                default_kp_[i] = 40.0f;
                default_kd_[i] = 1.0f;
            }
            break;
    }
}

void UnitreeInterface::InitializeDDS(const std::string& networkInterface) {
    // Initialize DDS
    ChannelFactory::Instance()->Init(0, networkInterface);
    
    // Create subscribers and publishers based on message type
    if (config_.message_type == MessageType::HG) {
        // HG message type
        lowstate_subscriber_ = std::make_shared<ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(HG_STATE_TOPIC);
        lowcmd_publisher_ = std::make_shared<ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>>(HG_CMD_TOPIC);
        
        auto hg_subscriber = std::static_pointer_cast<ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(lowstate_subscriber_);
        hg_subscriber->InitChannel(std::bind(&UnitreeInterface::LowStateHandler, this, std::placeholders::_1), 1);
        
        auto hg_publisher = std::static_pointer_cast<ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>>(lowcmd_publisher_);
        hg_publisher->InitChannel();
        
    } else {
        // GO2 message type
        lowstate_subscriber_ = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::LowState_>>(GO2_STATE_TOPIC);
        lowcmd_publisher_ = std::make_shared<ChannelPublisher<unitree_go::msg::dds_::LowCmd_>>(GO2_CMD_TOPIC);
        
        auto go2_subscriber = std::static_pointer_cast<ChannelSubscriber<unitree_go::msg::dds_::LowState_>>(lowstate_subscriber_);
        go2_subscriber->InitChannel(std::bind(&UnitreeInterface::LowStateHandler, this, std::placeholders::_1), 1);
        
        auto go2_publisher = std::static_pointer_cast<ChannelPublisher<unitree_go::msg::dds_::LowCmd_>>(lowcmd_publisher_);
        go2_publisher->InitChannel();
    }
    
    // Wireless controller subscriber (same for both message types)
    wireless_subscriber_ = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>>(TOPIC_JOYSTICK);
    auto wireless_sub = std::static_pointer_cast<ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>>(wireless_subscriber_);
    wireless_sub->InitChannel(std::bind(&UnitreeInterface::WirelessControllerHandler, this, std::placeholders::_1), 1);
    
    // Create command writer thread
    command_writer_ptr_ = CreateRecurrentThreadEx(
        "command_writer", UT_CPU_ID_NONE, 2000, &UnitreeInterface::LowCommandWriter, this);
    
    std::cout << "UnitreeInterface initialized: " << config_.name 
              << " (" << config_.num_motors << " motors, " 
              << (config_.message_type == MessageType::HG ? "HG" : "GO2") << " messages)"
              << " on interface: " << networkInterface << std::endl;
}

void UnitreeInterface::LowStateHandler(const void *message) {
    if (config_.message_type == MessageType::HG) {
        // Handle HG message
        unitree_hg::msg::dds_::LowState_ low_state = *(const unitree_hg::msg::dds_::LowState_ *)message;
        
        // CRC check
        if (low_state.crc() != Crc32Core((uint32_t *)&low_state, (sizeof(unitree_hg::msg::dds_::LowState_) >> 2) - 1)) {
            std::cout << "[ERROR] low_state CRC Error (HG)" << std::endl;
            return;
        }
        
        ProcessLowState(low_state);
        
    } else {
        // Handle GO2 message
        unitree_go::msg::dds_::LowState_ low_state = *(const unitree_go::msg::dds_::LowState_ *)message;
        
        // CRC check
        if (low_state.crc() != Crc32Core((uint32_t *)&low_state, (sizeof(unitree_go::msg::dds_::LowState_) >> 2) - 1)) {
            std::cout << "[ERROR] low_state CRC Error (GO2)" << std::endl;
            return;
        }
        
        ProcessLowState(low_state);
    }
}

void UnitreeInterface::ProcessLowState(const unitree_hg::msg::dds_::LowState_& low_state) {
    // Get motor state
    MotorState ms_tmp(config_.num_motors);
    for (int i = 0; i < config_.num_motors; ++i) {
        ms_tmp.q[i] = low_state.motor_state()[i].q();
        ms_tmp.dq[i] = low_state.motor_state()[i].dq();
        ms_tmp.tau_est[i] = low_state.motor_state()[i].tau_est();
        ms_tmp.temperature[i] = low_state.motor_state()[i].temperature()[0];
        ms_tmp.voltage[i] = low_state.motor_state()[i].vol();
    }
    motor_state_buffer_.SetData(ms_tmp);
    
    // Get IMU state
    ImuState imu_tmp;
    imu_tmp.omega = low_state.imu_state().gyroscope();
    imu_tmp.rpy = low_state.imu_state().rpy();
    imu_tmp.quat = low_state.imu_state().quaternion();
    imu_tmp.accel = low_state.imu_state().accelerometer();
    imu_state_buffer_.SetData(imu_tmp);
    
    // Update mode machine
    if (mode_machine_ != low_state.mode_machine()) {
        if (mode_machine_ == 0) {
            std::cout << config_.name << " type: " << unsigned(low_state.mode_machine()) << std::endl;
        }
        mode_machine_ = low_state.mode_machine();
    }
}

void UnitreeInterface::ProcessLowState(const unitree_go::msg::dds_::LowState_& low_state) {
    // Get motor state
    MotorState ms_tmp(config_.num_motors);
    for (int i = 0; i < config_.num_motors; ++i) {
        ms_tmp.q[i] = low_state.motor_state()[i].q();
        ms_tmp.dq[i] = low_state.motor_state()[i].dq();
        ms_tmp.tau_est[i] = low_state.motor_state()[i].tau_est();
        ms_tmp.temperature[i] = low_state.motor_state()[i].temperature(); // Single value, not array
        ms_tmp.voltage[i] = 0.0f; // GO2 doesn't have voltage field
    }
    motor_state_buffer_.SetData(ms_tmp);
    
    // Get IMU state
    ImuState imu_tmp;
    imu_tmp.omega = low_state.imu_state().gyroscope();
    imu_tmp.rpy = low_state.imu_state().rpy();
    imu_tmp.quat = low_state.imu_state().quaternion();
    imu_tmp.accel = low_state.imu_state().accelerometer();
    imu_state_buffer_.SetData(imu_tmp);
    
    // GO2 doesn't have mode_machine, keep current value
    // mode_machine_ remains unchanged
}

void UnitreeInterface::WirelessControllerHandler(const void *message) {
    std::lock_guard<std::mutex> lock(wireless_mutex_);
    
    unitree_go::msg::dds_::WirelessController_ wireless_msg = *(const unitree_go::msg::dds_::WirelessController_ *)message;
    
    PyWirelessController controller_tmp;
    
    controller_tmp.lx = wireless_msg.lx();
    controller_tmp.ly = wireless_msg.ly();
    controller_tmp.rx = wireless_msg.rx();
    controller_tmp.ry = wireless_msg.ry();
    controller_tmp.keys = wireless_msg.keys();
    
    wireless_controller_buffer_.SetData(controller_tmp);
}

void UnitreeInterface::LowCommandWriter() {
    const std::shared_ptr<const MotorCommand> mc = motor_command_buffer_.GetData();
    if (!mc) {
        static int no_cmd_counter = 0;
        if (no_cmd_counter % 1000 == 0) {
            std::cout << "[DEBUG] LowCommandWriter - No motor command available!" << std::endl;
        }
        no_cmd_counter++;
        return;
    }
    
    if (config_.message_type == MessageType::HG) {
        // Write HG command
        unitree_hg::msg::dds_::LowCmd_ dds_low_command;
        dds_low_command.mode_pr() = static_cast<uint8_t>(mode_);
        dds_low_command.mode_machine() = mode_machine_;
        
        for (size_t i = 0; i < config_.num_motors; i++) {
            dds_low_command.motor_cmd().at(i).mode() = 1;
            dds_low_command.motor_cmd().at(i).tau() = mc->tau_ff[i];
            dds_low_command.motor_cmd().at(i).q() = mc->q_target[i];
            dds_low_command.motor_cmd().at(i).dq() = mc->dq_target[i];
            dds_low_command.motor_cmd().at(i).kp() = mc->kp[i];
            dds_low_command.motor_cmd().at(i).kd() = mc->kd[i];
        }
        
        dds_low_command.crc() = Crc32Core((uint32_t *)&dds_low_command, (sizeof(dds_low_command) >> 2) - 1);
        
        auto hg_publisher = std::static_pointer_cast<ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>>(lowcmd_publisher_);
        hg_publisher->Write(dds_low_command);
        
    } else {
        // Write GO2 command
        unitree_go::msg::dds_::LowCmd_ dds_low_command;
        // dds_low_command.mode_pr() = static_cast<uint8_t>(mode_);
        // dds_low_command.mode_machine() = mode_machine_;
        
        for (size_t i = 0; i < config_.num_motors; i++) {
            dds_low_command.motor_cmd().at(i).mode() = 1;
            dds_low_command.motor_cmd().at(i).tau() = mc->tau_ff[i];
            dds_low_command.motor_cmd().at(i).q() = mc->q_target[i];
            dds_low_command.motor_cmd().at(i).dq() = mc->dq_target[i];
            dds_low_command.motor_cmd().at(i).kp() = mc->kp[i];
            dds_low_command.motor_cmd().at(i).kd() = mc->kd[i];
        }
        
        dds_low_command.crc() = Crc32Core((uint32_t *)&dds_low_command, (sizeof(dds_low_command) >> 2) - 1);
        
        auto go2_publisher = std::static_pointer_cast<ChannelPublisher<unitree_go::msg::dds_::LowCmd_>>(lowcmd_publisher_);
        go2_publisher->Write(dds_low_command);
    }
}

PyLowState UnitreeInterface::ConvertToPyLowState() {
    PyLowState py_state(config_.num_motors);
    
    const std::shared_ptr<const MotorState> ms = motor_state_buffer_.GetData();
    const std::shared_ptr<const ImuState> imu = imu_state_buffer_.GetData();
    
    if (ms) {
        py_state.motor.q = ms->q;
        py_state.motor.dq = ms->dq;
        py_state.motor.tau_est = ms->tau_est;
        py_state.motor.temperature = ms->temperature;
        py_state.motor.voltage = ms->voltage;
    }
    
    if (imu) {
        py_state.imu.rpy = imu->rpy;
        py_state.imu.omega = imu->omega;
        py_state.imu.quat = imu->quat;
        py_state.imu.accel = imu->accel;
    }
    
    py_state.mode_machine = mode_machine_;
    
    return py_state;
}

MotorCommand UnitreeInterface::ConvertFromPyMotorCommand(const PyMotorCommand& py_cmd) {
    MotorCommand cmd(config_.num_motors);
    cmd.q_target = py_cmd.q_target;
    cmd.dq_target = py_cmd.dq_target;
    cmd.kp = py_cmd.kp;
    cmd.kd = py_cmd.kd;
    cmd.tau_ff = py_cmd.tau_ff;
    return cmd;
}

// Python interface methods
PyLowState UnitreeInterface::ReadLowState() {
    return ConvertToPyLowState();
}

PyWirelessController UnitreeInterface::ReadWirelessController() {
    const std::shared_ptr<const PyWirelessController> controller = wireless_controller_buffer_.GetData();
    if (controller) {
        return *controller;
    } else {
        return PyWirelessController{};
    }
}

void UnitreeInterface::WriteLowCommand(const PyMotorCommand& command) {
    MotorCommand internal_cmd = ConvertFromPyMotorCommand(command);
    motor_command_buffer_.SetData(internal_cmd);
}

void UnitreeInterface::SetControlMode(PyControlMode mode) {
    mode_ = mode;
}

PyControlMode UnitreeInterface::GetControlMode() const {
    return mode_;
}

PyMotorCommand UnitreeInterface::CreateZeroCommand() {
    PyMotorCommand cmd(config_.num_motors);
    cmd.kp = default_kp_;
    cmd.kd = default_kd_;
    
    return cmd;
}

std::vector<float> UnitreeInterface::GetDefaultKp() const {
    return default_kp_;
}

std::vector<float> UnitreeInterface::GetDefaultKd() const {
    return default_kd_;
}

// Motion switcher (high-level service release). Lazy-init the client so
// callers who don't need it pay nothing, and so we can be sure the DDS
// factory has already been initialized by InitializeDDS.
std::string UnitreeInterface::CheckMotionMode() {
    if (!motion_switcher_) {
        motion_switcher_ = std::make_shared<unitree::robot::b2::MotionSwitcherClient>();
        motion_switcher_->SetTimeout(5.0f);
        motion_switcher_->Init();
    }
    std::string form, name;
    motion_switcher_->CheckMode(form, name);
    return name;
}

bool UnitreeInterface::ReleaseMotionControl(float timeout_sec) {
    if (!motion_switcher_) {
        motion_switcher_ = std::make_shared<unitree::robot::b2::MotionSwitcherClient>();
        motion_switcher_->SetTimeout(5.0f);
        motion_switcher_->Init();
    }
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(static_cast<int64_t>(timeout_sec * 1000.0f));
    std::string form, name;
    while (true) {
        motion_switcher_->CheckMode(form, name);
        if (name.empty()) {
            std::cout << "[MotionSwitcher] No active high-level mode; LowCmd writes will take effect." << std::endl;
            return true;
        }
        std::cout << "[MotionSwitcher] Active mode form='" << form
                  << "' name='" << name << "'. Calling ReleaseMode..." << std::endl;
        int32_t rc = motion_switcher_->ReleaseMode();
        if (rc != 0) {
            std::cout << "[MotionSwitcher] ReleaseMode returned " << rc
                      << " (will retry)." << std::endl;
        }
        if (std::chrono::steady_clock::now() >= deadline) {
            std::cout << "[MotionSwitcher] Timed out after " << timeout_sec
                      << "s waiting for release; mode still '" << name << "'." << std::endl;
            return false;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

// Static factory methods
std::shared_ptr<UnitreeInterface> UnitreeInterface::CreateG1(const std::string& networkInterface, MessageType message_type) {
    return std::make_shared<UnitreeInterface>(networkInterface, RobotType::G1, message_type);
}

std::shared_ptr<UnitreeInterface> UnitreeInterface::CreateH1(const std::string& networkInterface, MessageType message_type) {
    return std::make_shared<UnitreeInterface>(networkInterface, RobotType::H1, message_type);
}

std::shared_ptr<UnitreeInterface> UnitreeInterface::CreateH1_2(const std::string& networkInterface, MessageType message_type) {
    return std::make_shared<UnitreeInterface>(networkInterface, RobotType::H1_2, message_type);
}

std::shared_ptr<UnitreeInterface> UnitreeInterface::CreateCustom(const std::string& networkInterface, int num_motors, MessageType message_type) {
    return std::make_shared<UnitreeInterface>(networkInterface, RobotType::CUSTOM, message_type, num_motors);
}

// ===============================
// HandInterface Implementation
// ===============================

// Joint limits for Dex3-1 hands
static const std::array<float, DEX3_NUM_MOTORS> LEFT_HAND_MAX_LIMITS = 
    {1.0472f, 1.0472f, 1.74533f, 0.0f, 0.0f, 0.0f, 0.0f};
static const std::array<float, DEX3_NUM_MOTORS> LEFT_HAND_MIN_LIMITS = 
    {-1.0472f, -0.724312f, 0.0f, -1.5708f, -1.74533f, -1.5708f, -1.74533f};
static const std::array<float, DEX3_NUM_MOTORS> RIGHT_HAND_MAX_LIMITS = 
    {1.0472f, 0.724312f, 0.0f, 1.5708f, 1.74533f, 1.5708f, 1.74533f};
static const std::array<float, DEX3_NUM_MOTORS> RIGHT_HAND_MIN_LIMITS = 
    {-1.0472f, -1.0472f, -1.74533f, 0.0f, 0.0f, 0.0f, 0.0f};

// Default hand poses
static const std::array<float, DEX3_NUM_MOTORS> DEFAULT_LEFT_HAND_POSE = 
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static const std::array<float, DEX3_NUM_MOTORS> DEFAULT_RIGHT_HAND_POSE = 
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// RIS Mode structure for hand control
struct RISMode {
    uint8_t id     : 4;
    uint8_t status : 3;
    uint8_t timeout: 1;
    
    uint8_t ToUint8() const {
        uint8_t mode = 0;
        mode |= (id & 0x0F);
        mode |= (status & 0x07) << 4;
        mode |= (timeout & 0x01) << 7;
        return mode;
    }
};

// Constructor implementation
HandInterface::HandInterface(const std::string& networkInterface, HandType hand_type, bool re_init)
    : hand_type_(hand_type) {
    
    // Set hand name
    hand_name_ = (hand_type_ == HandType::LEFT_HAND) ? "LeftHand" : "RightHand";
    
    InitDefaultGains();
    InitializeDDS(networkInterface, re_init);
}

HandInterface::~HandInterface() {
    if (command_writer_ptr_) {
        command_writer_ptr_.reset();
    }
    hand_cmd_publisher_.reset();
    hand_state_subscriber_.reset();
}

void HandInterface::InitDefaultGains() {
    // Set default gains for hand motors
    for (int i = 0; i < DEX3_NUM_MOTORS; ++i) {
        default_kp_[i] = 0.5f;
        default_kd_[i] = 0.1f;
    }
}

void HandInterface::InitializeDDS(const std::string& networkInterface, bool re_init) {
    // Initialize DDS only if re_init is true
    if (re_init) {
        ChannelFactory::Instance()->Init(0, networkInterface);
    }
    
    // Set up topics based on hand type
    std::string cmd_topic, state_topic;
    if (hand_type_ == HandType::LEFT_HAND) {
        cmd_topic = LEFT_HAND_CMD_TOPIC;
        state_topic = LEFT_HAND_STATE_TOPIC;
    } else {
        cmd_topic = RIGHT_HAND_CMD_TOPIC;
        state_topic = RIGHT_HAND_STATE_TOPIC;
    }
    
    // Create publisher and subscriber
    hand_cmd_publisher_ = std::make_shared<ChannelPublisher<unitree_hg::msg::dds_::HandCmd_>>(cmd_topic);
    hand_state_subscriber_ = std::make_shared<ChannelSubscriber<unitree_hg::msg::dds_::HandState_>>(state_topic);
    
    // Initialize channels
    hand_cmd_publisher_->InitChannel();
    hand_state_subscriber_->InitChannel(std::bind(&HandInterface::HandStateHandler, this, std::placeholders::_1), 1);
    
    // Create command writer thread
    command_writer_ptr_ = CreateRecurrentThreadEx(
        "hand_command_writer", UT_CPU_ID_NONE, 2000, &HandInterface::HandCommandWriter, this);
    
    std::cout << "HandInterface initialized: " << hand_name_ 
              << " on interface: " << networkInterface 
              << " (re_init: " << (re_init ? "true" : "false") << ")" << std::endl;
}

void HandInterface::HandStateHandler(const void *message) {
    unitree_hg::msg::dds_::HandState_ hand_state = *(const unitree_hg::msg::dds_::HandState_ *)message;
    
    // Process motor state
    PyHandMotorState motor_state;
    for (size_t i = 0; i < DEX3_NUM_MOTORS && i < hand_state.motor_state().size(); ++i) {
        motor_state.q[i] = hand_state.motor_state()[i].q();
        motor_state.dq[i] = hand_state.motor_state()[i].dq();
        motor_state.tau_est[i] = hand_state.motor_state()[i].tau_est();
        
        // Handle temperature - it's an array in HandState
        if (hand_state.motor_state()[i].temperature().size() >= 2) {
            motor_state.temperature[i][0] = hand_state.motor_state()[i].temperature()[0];
            motor_state.temperature[i][1] = hand_state.motor_state()[i].temperature()[1];
        }
        
        motor_state.voltage[i] = hand_state.motor_state()[i].vol();
    }
    motor_state_buffer_.SetData(motor_state);
    
    // Process pressure sensor state
    PyHandPressSensorState press_sensor_state;
    for (size_t i = 0; i < DEX3_NUM_PRESS_SENSORS && i < hand_state.press_sensor_state().size(); ++i) {
        // Copy pressure data
        const auto& sensor = hand_state.press_sensor_state()[i];
        for (size_t j = 0; j < 12 && j < sensor.pressure().size(); ++j) {
            press_sensor_state.pressure[i][j] = sensor.pressure()[j];
        }
        for (size_t j = 0; j < 12 && j < sensor.temperature().size(); ++j) {
            press_sensor_state.temperature[i][j] = sensor.temperature()[j];
        }
        press_sensor_state.lost[i] = sensor.lost();
        
        // Reserve is a single uint32_t, not an array
        press_sensor_state.reserve[i][0] = sensor.reserve();
        // Fill remaining reserve slots with 0
        for (size_t j = 1; j < 4; ++j) {
            press_sensor_state.reserve[i][j] = 0;
        }
    }
    press_sensor_buffer_.SetData(press_sensor_state);
    
    // Process IMU state
    PyHandImuState imu_state;
    const auto& imu = hand_state.imu_state();
    if (imu.quaternion().size() >= 4) {
        for (int i = 0; i < 4; ++i) {
            imu_state.quaternion[i] = imu.quaternion()[i];
        }
    }
    if (imu.gyroscope().size() >= 3) {
        for (int i = 0; i < 3; ++i) {
            imu_state.gyroscope[i] = imu.gyroscope()[i];
        }
    }
    if (imu.accelerometer().size() >= 3) {
        for (int i = 0; i < 3; ++i) {
            imu_state.accelerometer[i] = imu.accelerometer()[i];
        }
    }
    if (imu.rpy().size() >= 3) {
        for (int i = 0; i < 3; ++i) {
            imu_state.rpy[i] = imu.rpy()[i];
        }
    }
    imu_state.temperature = imu.temperature();
    imu_buffer_.SetData(imu_state);
}

void HandInterface::HandCommandWriter() {
    const std::shared_ptr<const PyHandMotorCommand> cmd = motor_command_buffer_.GetData();
    if (!cmd) {
        static int no_cmd_counter = 0;
        if (no_cmd_counter % 1000 == 0) {
            std::cout << "[DEBUG] HandCommandWriter - No hand command available for " 
                      << hand_name_ << std::endl;
        }
        no_cmd_counter++;
        return;
    }
    
    // Create hand command message
    unitree_hg::msg::dds_::HandCmd_ hand_cmd;
    hand_cmd.motor_cmd().resize(DEX3_NUM_MOTORS);
    
    for (int i = 0; i < DEX3_NUM_MOTORS; ++i) {
        // Set RIS mode
        RISMode ris_mode;
        ris_mode.id = i;
        ris_mode.status = 0x01;  // Enable motor
        ris_mode.timeout = 0x00; // No timeout
        
        hand_cmd.motor_cmd()[i].mode() = ris_mode.ToUint8();
        hand_cmd.motor_cmd()[i].q() = cmd->q_target[i];
        hand_cmd.motor_cmd()[i].dq() = cmd->dq_target[i];
        hand_cmd.motor_cmd()[i].tau() = cmd->tau_ff[i];
        hand_cmd.motor_cmd()[i].kp() = cmd->kp[i];
        hand_cmd.motor_cmd()[i].kd() = cmd->kd[i];
    }
    
    hand_cmd_publisher_->Write(hand_cmd);
}

PyHandState HandInterface::ConvertToPyHandState() {
    PyHandState py_state;
    
    const std::shared_ptr<const PyHandMotorState> motor_state = motor_state_buffer_.GetData();
    const std::shared_ptr<const PyHandPressSensorState> press_sensor_state = press_sensor_buffer_.GetData();
    const std::shared_ptr<const PyHandImuState> imu_state = imu_buffer_.GetData();
    
    if (motor_state) {
        py_state.motor = *motor_state;
    }
    
    if (press_sensor_state) {
        py_state.press_sensor = *press_sensor_state;
    }
    
    if (imu_state) {
        py_state.imu = *imu_state;
    }
    
    return py_state;
}

const std::array<float, DEX3_NUM_MOTORS>& HandInterface::GetMaxLimits() const {
    return (hand_type_ == HandType::LEFT_HAND) ? LEFT_HAND_MAX_LIMITS : RIGHT_HAND_MAX_LIMITS;
}

const std::array<float, DEX3_NUM_MOTORS>& HandInterface::GetMinLimits() const {
    return (hand_type_ == HandType::LEFT_HAND) ? LEFT_HAND_MIN_LIMITS : RIGHT_HAND_MIN_LIMITS;
}

// Python interface methods
PyHandState HandInterface::ReadHandState() {
    return ConvertToPyHandState();
}

void HandInterface::WriteHandCommand(const PyHandMotorCommand& command) {
    motor_command_buffer_.SetData(command);
}

PyHandMotorCommand HandInterface::CreateZeroCommand() {
    PyHandMotorCommand cmd;
    cmd.kp = default_kp_;
    cmd.kd = default_kd_;
    // q_target, dq_target, and tau_ff are already initialized to zero
    return cmd;
}

PyHandMotorCommand HandInterface::CreateDefaultCommand() {
    PyHandMotorCommand cmd = CreateZeroCommand();
    
    // Set to default pose
    if (hand_type_ == HandType::LEFT_HAND) {
        cmd.q_target = DEFAULT_LEFT_HAND_POSE;
    } else {
        cmd.q_target = DEFAULT_RIGHT_HAND_POSE;
    }
    
    return cmd;
}

std::array<float, DEX3_NUM_MOTORS> HandInterface::GetDefaultKp() const {
    return default_kp_;
}

std::array<float, DEX3_NUM_MOTORS> HandInterface::GetDefaultKd() const {
    return default_kd_;
}

std::array<float, DEX3_NUM_MOTORS> HandInterface::GetMaxLimitsArray() {
    return GetMaxLimits();
}

std::array<float, DEX3_NUM_MOTORS> HandInterface::GetMinLimitsArray() {
    return GetMinLimits();
}

void HandInterface::ClampJointAngles(std::array<float, DEX3_NUM_MOTORS>& joint_angles) const {
    const auto& max_limits = GetMaxLimits();
    const auto& min_limits = GetMinLimits();
    
    for (int i = 0; i < DEX3_NUM_MOTORS; ++i) {
        joint_angles[i] = std::clamp(joint_angles[i], min_limits[i], max_limits[i]);
    }
}

std::array<float, DEX3_NUM_MOTORS> HandInterface::NormalizeJointAngles(const std::array<float, DEX3_NUM_MOTORS>& joint_angles) const {
    const auto& max_limits = GetMaxLimits();
    const auto& min_limits = GetMinLimits();
    
    std::array<float, DEX3_NUM_MOTORS> normalized;
    for (int i = 0; i < DEX3_NUM_MOTORS; ++i) {
        float range = max_limits[i] - min_limits[i];
        if (range > 0.0f) {
            normalized[i] = (joint_angles[i] - min_limits[i]) / range;
            normalized[i] = std::clamp(normalized[i], 0.0f, 1.0f);
        } else {
            normalized[i] = 0.0f;
        }
    }
    return normalized;
}

// Static factory methods
std::shared_ptr<HandInterface> HandInterface::CreateLeftHand(const std::string& networkInterface, bool re_init) {
    return std::make_shared<HandInterface>(networkInterface, HandType::LEFT_HAND, re_init);
}

std::shared_ptr<HandInterface> HandInterface::CreateRightHand(const std::string& networkInterface, bool re_init) {
    return std::make_shared<HandInterface>(networkInterface, HandType::RIGHT_HAND, re_init);
}

// =====================================================================
// Dex1Interface — 1-DOF parallel-jaw gripper bridged via dex1_1_service
// =====================================================================

Dex1Interface::Dex1Interface(const std::string& networkInterface, Dex1Type side, bool re_init)
    : side_(side) {
    side_name_ = (side_ == Dex1Type::LEFT) ? "Dex1Left" : "Dex1Right";
    InitDefaultGains();
    InitializeDDS(networkInterface, re_init);
}

Dex1Interface::~Dex1Interface() {
    if (command_writer_ptr_) {
        command_writer_ptr_.reset();
    }
    cmd_publisher_.reset();
    state_subscriber_.reset();
}

void Dex1Interface::InitDefaultGains() {
    // Defaults match the values in dex1_1_service/test/test_gripper.cpp.
    default_kp_ = 5.0f;
    default_kd_ = 0.05f;
}

void Dex1Interface::InitializeDDS(const std::string& networkInterface, bool re_init) {
    if (re_init) {
        ChannelFactory::Instance()->Init(0, networkInterface);
    }

    const std::string cmd_topic = (side_ == Dex1Type::LEFT) ? LEFT_DEX1_CMD_TOPIC : RIGHT_DEX1_CMD_TOPIC;
    const std::string state_topic = (side_ == Dex1Type::LEFT) ? LEFT_DEX1_STATE_TOPIC : RIGHT_DEX1_STATE_TOPIC;

    cmd_publisher_ = std::make_shared<ChannelPublisher<unitree_go::msg::dds_::MotorCmds_>>(cmd_topic);
    state_subscriber_ = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::MotorStates_>>(state_topic);

    cmd_publisher_->InitChannel();
    state_subscriber_->InitChannel(std::bind(&Dex1Interface::StateHandler, this, std::placeholders::_1), 1);

    // 200 Hz republish — matches the cadence used by dex1_1_service's own
    // test_gripper.cpp client. The service runs its serial loop at 1 kHz
    // internally, so this rate keeps it well-fed without saturating DDS.
    command_writer_ptr_ = CreateRecurrentThreadEx(
        "dex1_command_writer", UT_CPU_ID_NONE, 5000, &Dex1Interface::CommandWriter, this);

    std::cout << "Dex1Interface initialized: " << side_name_
              << " on interface: " << networkInterface
              << " (re_init: " << (re_init ? "true" : "false") << ")" << std::endl;
}

void Dex1Interface::StateHandler(const void *message) {
    const auto& msg = *(const unitree_go::msg::dds_::MotorStates_ *)message;
    if (msg.states().empty()) {
        return;
    }
    PyDex1State s;
    const auto& m0 = msg.states()[0];
    s.q = m0.q();
    s.dq = m0.dq();
    s.tau_est = m0.tau_est();
    s.temperature = m0.temperature();
    state_buffer_.SetData(s);
}

void Dex1Interface::CommandWriter() {
    const std::shared_ptr<const PyDex1Command> cmd = command_buffer_.GetData();
    if (!cmd) {
        return;
    }

    unitree_go::msg::dds_::MotorCmds_ msg;
    msg.cmds().resize(1);
    auto& m0 = msg.cmds()[0];
    m0.mode() = cmd->mode;
    m0.q() = cmd->q_target;
    m0.dq() = cmd->dq_target;
    m0.tau() = cmd->tau_ff;
    m0.kp() = cmd->kp;
    m0.kd() = cmd->kd;

    cmd_publisher_->Write(msg);
}

PyDex1State Dex1Interface::ReadState() {
    const std::shared_ptr<const PyDex1State> s = state_buffer_.GetData();
    return s ? *s : PyDex1State{};
}

void Dex1Interface::WriteCommand(const PyDex1Command& command) {
    command_buffer_.SetData(command);
}

PyDex1Command Dex1Interface::CreateZeroCommand() {
    PyDex1Command cmd;
    cmd.kp = default_kp_;
    cmd.kd = default_kd_;
    return cmd;
}

std::shared_ptr<Dex1Interface> Dex1Interface::CreateLeft(const std::string& networkInterface, bool re_init) {
    return std::make_shared<Dex1Interface>(networkInterface, Dex1Type::LEFT, re_init);
}

std::shared_ptr<Dex1Interface> Dex1Interface::CreateRight(const std::string& networkInterface, bool re_init) {
    return std::make_shared<Dex1Interface>(networkInterface, Dex1Type::RIGHT, re_init);
}
