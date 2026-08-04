#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include "unitree_interface.hpp"

namespace py = pybind11;

PYBIND11_MODULE(unitree_interface, m) {
    m.doc() = "General Unitree robot interface supporting both HG and GO2 message types";
    
    // Enums
    py::enum_<RobotType>(m, "RobotType")
        .value("G1", RobotType::G1)
        .value("H1", RobotType::H1)
        .value("H1_2", RobotType::H1_2)
        .value("CUSTOM", RobotType::CUSTOM)
        .export_values();
    
    py::enum_<MessageType>(m, "MessageType")
        .value("HG", MessageType::HG)
        .value("GO2", MessageType::GO2)
        .export_values();
    
    py::enum_<PyControlMode>(m, "ControlMode")
        .value("PR", PyControlMode::PR)
        .value("AB", PyControlMode::AB)
        .export_values();
    
    py::enum_<HandType>(m, "HandType")
        .value("LEFT_HAND", HandType::LEFT_HAND)
        .value("RIGHT_HAND", HandType::RIGHT_HAND)
        .export_values();

    py::enum_<Dex1Type>(m, "Dex1Type")
        .value("LEFT", Dex1Type::LEFT)
        .value("RIGHT", Dex1Type::RIGHT)
        .export_values();
    
    // Data structures
    py::class_<PyImuState>(m, "ImuState")
        .def(py::init<>())
        .def_readwrite("rpy", &PyImuState::rpy)
        .def_readwrite("omega", &PyImuState::omega)
        .def_readwrite("quat", &PyImuState::quat)
        .def_readwrite("accel", &PyImuState::accel);
    
    py::class_<PyMotorState>(m, "MotorState")
        .def(py::init<int>())
        .def_readwrite("q", &PyMotorState::q)
        .def_readwrite("dq", &PyMotorState::dq)
        .def_readwrite("tau_est", &PyMotorState::tau_est)
        .def_readwrite("temperature", &PyMotorState::temperature)
        .def_readwrite("voltage", &PyMotorState::voltage);
    
    py::class_<PyMotorCommand>(m, "MotorCommand")
        .def(py::init<int>())
        .def_readwrite("q_target", &PyMotorCommand::q_target)
        .def_readwrite("dq_target", &PyMotorCommand::dq_target)
        .def_readwrite("kp", &PyMotorCommand::kp)
        .def_readwrite("kd", &PyMotorCommand::kd)
        .def_readwrite("tau_ff", &PyMotorCommand::tau_ff);
    
    py::class_<PyWirelessController>(m, "WirelessController")
        .def(py::init<>())
        .def_readwrite("lx", &PyWirelessController::lx)
        .def_readwrite("ly", &PyWirelessController::ly)
        .def_readwrite("rx", &PyWirelessController::rx)
        .def_readwrite("ry", &PyWirelessController::ry)
        .def_readwrite("keys", &PyWirelessController::keys);
    
    py::class_<PyLowState>(m, "LowState")
        .def(py::init<int>())
        .def_readwrite("imu", &PyLowState::imu)
        .def_readwrite("imu_torso", &PyLowState::imu_torso)
        .def_readwrite("imu_torso_valid", &PyLowState::imu_torso_valid)
        .def_readwrite("motor", &PyLowState::motor)
        .def_readwrite("mode_machine", &PyLowState::mode_machine);
    
    py::class_<RobotConfig>(m, "RobotConfig")
        .def(py::init<RobotType, MessageType, int, const std::string&>())
        .def_readwrite("robot_type", &RobotConfig::robot_type)
        .def_readwrite("message_type", &RobotConfig::message_type)
        .def_readwrite("num_motors", &RobotConfig::num_motors)
        .def_readwrite("name", &RobotConfig::name);
    
    // Hand data structures
    py::class_<PyHandMotorState>(m, "HandMotorState")
        .def(py::init<>())
        .def_readwrite("q", &PyHandMotorState::q)
        .def_readwrite("dq", &PyHandMotorState::dq)
        .def_readwrite("tau_est", &PyHandMotorState::tau_est)
        .def_readwrite("temperature", &PyHandMotorState::temperature)
        .def_readwrite("voltage", &PyHandMotorState::voltage);

    py::class_<PyHandMotorCommand>(m, "HandMotorCommand")
        .def(py::init<>())
        .def_readwrite("q_target", &PyHandMotorCommand::q_target)
        .def_readwrite("dq_target", &PyHandMotorCommand::dq_target)
        .def_readwrite("kp", &PyHandMotorCommand::kp)
        .def_readwrite("kd", &PyHandMotorCommand::kd)
        .def_readwrite("tau_ff", &PyHandMotorCommand::tau_ff);

    py::class_<PyHandPressSensorState>(m, "HandPressSensorState")
        .def(py::init<>())
        .def_readwrite("pressure", &PyHandPressSensorState::pressure)
        .def_readwrite("temperature", &PyHandPressSensorState::temperature)
        .def_readwrite("lost", &PyHandPressSensorState::lost)
        .def_readwrite("reserve", &PyHandPressSensorState::reserve);

    py::class_<PyHandImuState>(m, "HandImuState")
        .def(py::init<>())
        .def_readwrite("quaternion", &PyHandImuState::quaternion)
        .def_readwrite("gyroscope", &PyHandImuState::gyroscope)
        .def_readwrite("accelerometer", &PyHandImuState::accelerometer)
        .def_readwrite("rpy", &PyHandImuState::rpy)
        .def_readwrite("temperature", &PyHandImuState::temperature);

    py::class_<PyHandState>(m, "HandState")
        .def(py::init<>())
        .def_readwrite("motor", &PyHandState::motor)
        .def_readwrite("press_sensor", &PyHandState::press_sensor)
        .def_readwrite("imu", &PyHandState::imu)
        .def_readwrite("power_v", &PyHandState::power_v)
        .def_readwrite("power_a", &PyHandState::power_a)
        .def_readwrite("system_v", &PyHandState::system_v)
        .def_readwrite("device_v", &PyHandState::device_v)
        .def_readwrite("error", &PyHandState::error)
        .def_readwrite("reserve", &PyHandState::reserve);

    // Dex1-1 single-motor structs (1-DOF parallel jaw)
    py::class_<PyDex1State>(m, "Dex1State")
        .def(py::init<>())
        .def_readwrite("q", &PyDex1State::q)
        .def_readwrite("dq", &PyDex1State::dq)
        .def_readwrite("tau_est", &PyDex1State::tau_est)
        .def_readwrite("temperature", &PyDex1State::temperature);

    py::class_<PyDex1Command>(m, "Dex1Command")
        .def(py::init<>())
        .def_readwrite("mode", &PyDex1Command::mode)
        .def_readwrite("q_target", &PyDex1Command::q_target)
        .def_readwrite("dq_target", &PyDex1Command::dq_target)
        .def_readwrite("kp", &PyDex1Command::kp)
        .def_readwrite("kd", &PyDex1Command::kd)
        .def_readwrite("tau_ff", &PyDex1Command::tau_ff);
    
    // Main interface class
    py::class_<UnitreeInterface, std::shared_ptr<UnitreeInterface>>(m, "UnitreeInterface")
        // Constructors
        .def(py::init<const std::string&, RobotType, MessageType>())
        .def(py::init<const std::string&, const RobotConfig&>())
        .def(py::init<const std::string&, RobotType, MessageType, int>())
        
        // Python interface methods
        .def("read_low_state", &UnitreeInterface::ReadLowState)
        .def("read_wireless_controller", &UnitreeInterface::ReadWirelessController)
        .def("write_low_command", static_cast<void(UnitreeInterface::*)(const PyMotorCommand&)>(&UnitreeInterface::WriteLowCommand))
        .def("set_control_mode", &UnitreeInterface::SetControlMode)
        .def("get_control_mode", &UnitreeInterface::GetControlMode)
        
        // Utility methods
        .def("create_zero_command", &UnitreeInterface::CreateZeroCommand)
        .def("get_default_kp", &UnitreeInterface::GetDefaultKp)
        .def("get_default_kd", &UnitreeInterface::GetDefaultKd)

        // Motion switcher (release high-level service before LowCmd takes effect)
        .def("release_motion_control", &UnitreeInterface::ReleaseMotionControl,
             py::arg("timeout_sec") = 30.0f,
             "Release any active high-level motion control service so LowCmd "
             "writes take effect. Polls CheckMode/ReleaseMode at 1 Hz until no "
             "mode is active or the timeout elapses. Returns True on success.")
        .def("check_motion_mode", &UnitreeInterface::CheckMotionMode,
             "Return the currently-active high-level motion mode name, or an "
             "empty string when no service is running.")
        
        // Configuration methods
        .def("get_config", &UnitreeInterface::GetConfig)
        .def("get_num_motors", &UnitreeInterface::GetNumMotors)
        .def("get_robot_name", static_cast<std::string(UnitreeInterface::*)() const>(&UnitreeInterface::GetRobotName))
        
        // Static factory methods
        .def_static("create_g1", &UnitreeInterface::CreateG1, 
                   py::arg("network_interface"), py::arg("message_type") = MessageType::HG)
        .def_static("create_h1", &UnitreeInterface::CreateH1,
                   py::arg("network_interface"), py::arg("message_type") = MessageType::GO2)
        .def_static("create_h1_2", &UnitreeInterface::CreateH1_2,
                   py::arg("network_interface"), py::arg("message_type") = MessageType::HG)
        .def_static("create_custom", &UnitreeInterface::CreateCustom,
                   py::arg("network_interface"), py::arg("num_motors"), 
                   py::arg("message_type") = MessageType::HG);
    
    // Predefined configurations (expose as module attributes since RobotConfigs is a namespace)
    m.attr("G1_HG_CONFIG") = RobotConfigs::G1_HG;
    m.attr("H1_GO2_CONFIG") = RobotConfigs::H1_GO2;
    m.attr("H1_2_HG_CONFIG") = RobotConfigs::H1_2_HG;
    
    // Module-level functions for convenience
    m.def("create_robot", [](const std::string& network_interface, RobotType robot_type, 
                            MessageType message_type = MessageType::HG) {
        switch (robot_type) {
            case RobotType::G1: return UnitreeInterface::CreateG1(network_interface, message_type);
            case RobotType::H1: return UnitreeInterface::CreateH1(network_interface, message_type);
            case RobotType::H1_2: return UnitreeInterface::CreateH1_2(network_interface, message_type);
            default: throw std::runtime_error("Unknown robot type");
        }
    }, py::arg("network_interface"), py::arg("robot_type"), py::arg("message_type") = MessageType::HG);
    
    m.def("create_robot_with_config", [](const std::string& network_interface, const RobotConfig& config) {
        return std::make_shared<UnitreeInterface>(network_interface, config);
    }, py::arg("network_interface"), py::arg("config"));
    
    // Hand Interface Class
    py::class_<HandInterface, std::shared_ptr<HandInterface>>(m, "HandInterface")
        // Constructor
        .def(py::init<const std::string&, HandType, bool>(), 
             py::arg("network_interface"), py::arg("hand_type"), py::arg("re_init") = true,
             "Initialize hand interface with network interface, hand type, and optional re_init flag")
        
        // Core interface methods
        .def("read_hand_state", &HandInterface::ReadHandState,
             "Read current hand state including motor positions, sensors, and IMU data")
        .def("write_hand_command", &HandInterface::WriteHandCommand,
             "Send motor commands to the hand")
        
        // Utility methods
        .def("create_zero_command", &HandInterface::CreateZeroCommand,
             "Create a zero command with default gains")
        .def("create_default_command", &HandInterface::CreateDefaultCommand,
             "Create a command with default pose and gains")
        .def("get_default_kp", &HandInterface::GetDefaultKp,
             "Get default proportional gains for all joints")
        .def("get_default_kd", &HandInterface::GetDefaultKd,
             "Get default derivative gains for all joints")
        .def("get_max_limits", &HandInterface::GetMaxLimitsArray,
             "Get maximum joint angle limits")
        .def("get_min_limits", &HandInterface::GetMinLimitsArray,
             "Get minimum joint angle limits")
        
        // Configuration methods
        .def("get_hand_type", &HandInterface::GetHandType,
             "Get the hand type (LEFT_HAND or RIGHT_HAND)")
        .def("get_hand_name", &HandInterface::GetHandName,
             "Get the hand name string")
        
        // Joint manipulation utilities
        .def("clamp_joint_angles", &HandInterface::ClampJointAngles,
             "Clamp joint angles to valid range in-place")
        .def("normalize_joint_angles", &HandInterface::NormalizeJointAngles,
             "Normalize joint angles to [0, 1] range based on joint limits")
        
        // Static factory methods
        .def_static("create_left_hand", &HandInterface::CreateLeftHand,
                   py::arg("network_interface"), py::arg("re_init") = true,
                   "Create a left hand interface")
        .def_static("create_right_hand", &HandInterface::CreateRightHand,
                   py::arg("network_interface"), py::arg("re_init") = true,
                   "Create a right hand interface");
    
    // Hand module-level convenience functions
    m.def("create_hand", [](const std::string& network_interface, HandType hand_type, bool re_init = true) {
        if (hand_type == HandType::LEFT_HAND) {
            return HandInterface::CreateLeftHand(network_interface, re_init);
        } else {
            return HandInterface::CreateRightHand(network_interface, re_init);
        }
    }, py::arg("network_interface"), py::arg("hand_type"), py::arg("re_init") = true,
       "Create a hand interface based on hand type");
    
    m.def("create_dual_hands", [](const std::string& network_interface, bool re_init = true) {
        auto left_hand = HandInterface::CreateLeftHand(network_interface, re_init);
        auto right_hand = HandInterface::CreateRightHand(network_interface, false); // Don't re-init for second hand
        return py::make_tuple(left_hand, right_hand);
    }, py::arg("network_interface"), py::arg("re_init") = true,
       "Create both left and right hand interfaces (left hand initializes DDS, right hand uses existing connection)");
    
    // Constants
    m.attr("G1_NUM_MOTOR") = 29;
    m.attr("H1_NUM_MOTOR") = 19;
    m.attr("H1_2_NUM_MOTOR") = 29;
    
    // Hand constants
    m.attr("DEX3_NUM_MOTORS") = DEX3_NUM_MOTORS;
    m.attr("DEX3_NUM_PRESS_SENSORS") = DEX3_NUM_PRESS_SENSORS;
    
    // Hand topic names
    m.attr("LEFT_HAND_CMD_TOPIC") = LEFT_HAND_CMD_TOPIC;
    m.attr("LEFT_HAND_STATE_TOPIC") = LEFT_HAND_STATE_TOPIC;
    m.attr("RIGHT_HAND_CMD_TOPIC") = RIGHT_HAND_CMD_TOPIC;
    m.attr("RIGHT_HAND_STATE_TOPIC") = RIGHT_HAND_STATE_TOPIC;

    // Dex1-1 gripper interface
    py::class_<Dex1Interface, std::shared_ptr<Dex1Interface>>(m, "Dex1Interface")
        .def(py::init<const std::string&, Dex1Type, bool>(),
             py::arg("network_interface"), py::arg("side"), py::arg("re_init") = true,
             "Initialize Dex1-1 gripper interface for one side")

        .def("read_state", &Dex1Interface::ReadState,
             "Read latest single-motor gripper state (q, dq, tau_est, temperature)")
        .def("write_command", &Dex1Interface::WriteCommand,
             "Buffer a single-motor command; the writer thread republishes at 200 Hz")
        .def("create_zero_command", &Dex1Interface::CreateZeroCommand,
             "Create a zero-target command preloaded with default kp/kd")

        .def("get_default_kp", &Dex1Interface::GetDefaultKp)
        .def("get_default_kd", &Dex1Interface::GetDefaultKd)
        .def("get_side", &Dex1Interface::GetSide)
        .def("get_side_name", &Dex1Interface::GetSideName)

        .def_static("create_left", &Dex1Interface::CreateLeft,
                   py::arg("network_interface"), py::arg("re_init") = true)
        .def_static("create_right", &Dex1Interface::CreateRight,
                   py::arg("network_interface"), py::arg("re_init") = true);

    m.def("create_dex1_pair", [](const std::string& network_interface, bool re_init = true) {
        auto left = Dex1Interface::CreateLeft(network_interface, re_init);
        auto right = Dex1Interface::CreateRight(network_interface, false);
        return py::make_tuple(left, right);
    }, py::arg("network_interface"), py::arg("re_init") = true,
       "Create both left and right Dex1-1 gripper interfaces. Left initializes "
       "DDS (unless re_init=False because the caller already did), right reuses it.");

    // Dex1-1 topic names
    m.attr("LEFT_DEX1_CMD_TOPIC") = LEFT_DEX1_CMD_TOPIC;
    m.attr("LEFT_DEX1_STATE_TOPIC") = LEFT_DEX1_STATE_TOPIC;
    m.attr("RIGHT_DEX1_CMD_TOPIC") = RIGHT_DEX1_CMD_TOPIC;
    m.attr("RIGHT_DEX1_STATE_TOPIC") = RIGHT_DEX1_STATE_TOPIC;
}
