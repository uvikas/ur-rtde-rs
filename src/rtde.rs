/* RTDE Protocol */

use crate::robot_state::{RobotState, StateDataTypes};
use log::{debug, info};
use socket2::{Domain, Socket, Type};
use std::io::{Read, Write};
use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Duration;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;
use tokio::sync::Mutex;
use tokio::time::Instant;

use crate::utils::{
    double2hexstr, get_bool, get_double, get_i32, get_u32, get_u64, get_u8, hex2bytes, pack_double,
    pack_int32, pack_uint32, pack_vector_n_double, pack_vector_n_int32, read_rtde_header,
    unpack_vector3d, unpack_vector6_i32, unpack_vector6d,
};

const RTDE_PROTOCOL_VERSION: u8 = 2;
const HEADER_SIZE: u16 = 3;
pub const RTDE_START_SYNCHRONIZATION_TIMEOUT: u64 = 5;

// All RTDE fields for e-series UR
pub const RTDE_FIELDS: &[&str] = &[
    "timestamp",
    "target_q",
    "target_qd",
    "target_qdd",
    "target_current",
    "target_moment",
    "actual_q",
    "actual_qd",
    "actual_current",
    "joint_control_output",
    "actual_TCP_pose",
    "actual_TCP_speed",
    "actual_TCP_force",
    "target_TCP_pose",
    "target_TCP_speed",
    "actual_digital_input_bits",
    "joint_temperatures",
    "actual_execution_time",
    "robot_mode",
    "joint_mode",
    "safety_mode",
    "actual_tool_accelerometer",
    "speed_scaling",
    "target_speed_fraction",
    "actual_momentum",
    "actual_main_voltage",
    "actual_robot_voltage",
    "actual_robot_current",
    "actual_joint_voltage",
    "actual_digital_output_bits",
    "runtime_state",
    "standard_analog_input0",
    "standard_analog_input1",
    "standard_analog_output0",
    "standard_analog_output1",
    "robot_status_bits",
    "safety_status_bits",
    "ft_raw_wrench",
    "payload",
    "payload_cog",
    "payload_inertia",
    "output_int_register_2",
    "output_int_register_12",
    "output_int_register_13",
    "output_int_register_14",
    "output_int_register_15",
    "output_int_register_16",
    "output_int_register_17",
    "output_int_register_18",
    "output_int_register_19",
    "output_double_register_12",
    "output_double_register_13",
    "output_double_register_14",
    "output_double_register_15",
    "output_double_register_16",
    "output_double_register_17",
    "output_double_register_18",
    "output_double_register_19",
];

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[repr(u8)]
pub enum CommandType {
    NoCmd = 0,
    MoveJ = 1,
    MoveJIK = 2,
    MoveL = 3,
    MoveLFK = 4,
    ForceMode = 6,
    ForceModeStop = 7,
    ZeroFtSensor = 8,
    SpeedJ = 9,
    SpeedL = 10,
    ServoJ = 11,
    ServoC = 12,
    SetStdDigitalOut = 13,
    SetToolDigitalOut = 14,
    SpeedStop = 15,
    ServoStop = 16,
    SetPayload = 17,
    TeachMode = 18,
    EndTeachMode = 19,
    ForceModeSetDamping = 20,
    ForceModeSetGainScaling = 21,
    SetSpeedSlider = 22,
    SetStdAnalogOut = 23,
    ServoL = 24,
    ToolContact = 25,
    GetSteptime = 26,
    GetActualJointPositionsHistory = 27,
    GetTargetWaypoint = 28,
    SetTcp = 29,
    GetInverseKinematicsArgs = 30,
    ProtectiveStop = 31,
    StopL = 33,
    StopJ = 34,
    SetWatchdog = 35,
    IsPoseWithinSafetyLimits = 36,
    IsJointsWithinSafetyLimits = 37,
    GetJointTorques = 38,
    PoseTrans = 39,
    GetTcpOffset = 40,
    JogStart = 41,
    JogStop = 42,
    GetForwardKinematicsDefault = 43,
    GetForwardKinematicsArgs = 44,
    MovePath = 45,
    GetInverseKinematicsDefault = 46,
    IsSteady = 47,
    SetConfDigitalOut = 48,
    SetInputIntRegister = 49,
    SetInputDoubleRegister = 50,
    MoveUntilContact = 51,
    FreedriveMode = 52,
    EndFreedriveMode = 53,
    GetFreedriveStatus = 54,
    SetExternalForceTorque = 55,
    FtRtdeInputEnable = 56,
    EnableExternalFtSensor = 57,
    GetActualToolFlangePose = 58,
    SetGravity = 59,
    GetInverseKinematicsHasSolutionDefault = 60,
    GetInverseKinematicsHasSolutionArgs = 61,
    StartContactDetection = 62,
    StopContactDetection = 63,
    ReadContactDetection = 64,
    SetTargetPayload = 65,
    Watchdog = 99,
    StopScript = 255,
}

#[derive(Debug)]
#[repr(u8)]
#[allow(dead_code)]
pub enum Recipe {
    Recipe1 = 1,
    Recipe2 = 2,
    Recipe3 = 3,
    Recipe4 = 4,
    Recipe5 = 5,
    Recipe6 = 6,
    Recipe7 = 7,
    Recipe8 = 8,
    Recipe9 = 9,
    Recipe10 = 10,
    Recipe11 = 11,
    Recipe12 = 12,
    Recipe13 = 13,
    Recipe14 = 14,
    Recipe15 = 15,
    Recipe16 = 16,
    Recipe17 = 17,
    Recipe18 = 18,
    Recipe19 = 19,
    Recipe20 = 20,
    Recipe21 = 21,
}

#[repr(u32)]
pub enum SafetyStatusBits {
    IsNormalMode = 0,
    IsReducedMode = 1,
    IsProtectiveStopped = 2,
    IsRecoveryMode = 3,
    IsSafeguardStopped = 4,
    IsSystemEmergencyStopped = 5,
    IsRobotEmergencyStopped = 6,
    IsEmergencyStopped = 7,
    IsViolation = 8,
    IsFault = 9,
    IsStoppedDueToSafety = 10,
}

#[derive(Debug)]
#[allow(dead_code)]
pub struct RobotCommand {
    pub command_type: CommandType,
    recipe: Recipe,
    async_: Option<i32>,
    ft_rtde_input_enable: Option<i32>,
    reg_int_val: Option<i32>,
    reg_double_val: Option<f64>,
    val: Option<Vec<f64>>,
    selection_vector: Option<Vec<i32>>,
    free_axes: Option<Vec<i32>>,
    force_mode_type: Option<i32>,
    std_digital_out: Option<u8>,
    std_digital_out_mask: Option<u8>,
    configurable_digital_out: Option<u8>,
    configurable_digital_out_mask: Option<u8>,
    std_tool_out: Option<u8>,
    std_tool_out_mask: Option<u8>,
    std_analog_output_mask: Option<u8>,
    std_analog_output_type: Option<u8>,
    std_analog_output_0: Option<f64>,
    std_analog_output_1: Option<f64>,
    speed_slider_mask: Option<i32>,
    speed_slider_fraction: Option<f64>,
    steps: Option<u32>,
}

impl RobotCommand {
    pub fn new(command_type: CommandType, recipe: Recipe) -> Self {
        Self {
            command_type,
            recipe,
            async_: None,
            ft_rtde_input_enable: None,
            reg_int_val: None,
            reg_double_val: None,
            val: None,
            selection_vector: None,
            free_axes: None,
            force_mode_type: None,
            std_digital_out: None,
            std_digital_out_mask: None,
            configurable_digital_out: None,
            configurable_digital_out_mask: None,
            std_tool_out: None,
            std_tool_out_mask: None,
            std_analog_output_mask: None,
            std_analog_output_type: None,
            std_analog_output_0: None,
            std_analog_output_1: None,
            speed_slider_mask: None,
            speed_slider_fraction: None,
            steps: None,
        }
    }
}

pub trait RTDETrait {}

#[derive(Debug, PartialEq)]
#[repr(u32)]
enum RTDECommand {
    RtdeRequestProtocolVersion = 86,
    RtdeGetUrcontrolVersion = 118,
    RtdeTextMessage = 77,
    RtdeDataPackage = 85,
    RtdeControlPackageSetupOutputs = 79,
    RtdeControlPackageSetupInputs = 73,
    RtdeControlPackageStart = 83,
    RtdeControlPackagePause = 80,
}

#[derive(Debug, PartialEq)]
#[repr(u8)]

enum ConnectionState {
    Disconnected = 0,
    Connected = 1,
    Started = 2,
}

#[derive(Debug)]
pub enum RTDEError {
    ConnectionError(String),
    ProtocolError(String),
    StateError(String),
    NoDataAvailable(String),
    RobotNotInRemoteControl(String),
    RobotConnectionTimeout(String),
    ScriptClientError(String),
    DashboardError(String),
}

impl std::error::Error for RTDEError {}

impl std::fmt::Display for RTDEError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            Self::ConnectionError(msg) => write!(f, "Connection error: {}", msg),
            Self::ProtocolError(msg) => write!(f, "Protocol error: {}", msg),
            Self::StateError(msg) => write!(f, "State error: {}", msg),
            Self::NoDataAvailable(msg) => write!(f, "No data available: {}", msg),
            Self::RobotNotInRemoteControl(msg) => write!(f, "Robot not in remote control: {}", msg),
            Self::DashboardError(msg) => write!(f, "Dashboard error: {}", msg),
            Self::RobotConnectionTimeout(msg) => write!(f, "Robot connection timeout: {}", msg),
            Self::ScriptClientError(msg) => write!(f, "Script client error: {}", msg),
        }
    }
}

impl From<std::io::Error> for RTDEError {
    fn from(err: std::io::Error) -> Self {
        RTDEError::ConnectionError(err.to_string())
    }
}

impl From<String> for RTDEError {
    fn from(err: String) -> Self {
        RTDEError::ConnectionError(err)
    }
}

pub struct RTDE {
    hostname: String,
    port: u16,
    verbose: bool,
    conn_state: ConnectionState,
    output_types: Vec<String>,
    output_names: Vec<String>,
    stream: Option<tokio::net::TcpStream>,
    buffer: Vec<u8>,
    deadline: Instant,
}

impl RTDE {
    pub fn new(hostname: &str) -> Self {
        Self {
            hostname: hostname.to_string(),
            port: 30004,
            verbose: false,
            conn_state: ConnectionState::Disconnected,
            output_types: Vec::new(),
            output_names: Vec::new(),
            stream: None,
            buffer: Vec::new(),
            deadline: Instant::now(),
        }
    }

    pub async fn connect(&mut self) -> Result<(), RTDEError> {
        self.buffer.clear();
        let addr = format!("{}:{}", self.hostname, self.port)
            .parse::<SocketAddr>()
            .map_err(|e| format!("Invalid address: {}", e))?;

        let stream = TcpStream::connect(&addr).await?;
        stream.set_nodelay(true)?;

        self.stream = Some(stream);
        self.conn_state = ConnectionState::Connected;

        debug!("Connected to robot!");

        Ok(())
    }

    pub async fn disconnect(&mut self) -> Result<(), RTDEError> {
        if let Some(mut stream) = self.stream.take() {
            stream.shutdown().await?;
        }
        self.conn_state = ConnectionState::Disconnected;
        debug!("Disconnected from robot");
        Ok(())
    }

    pub fn is_connected(&self) -> bool {
        self.conn_state == ConnectionState::Connected || self.conn_state == ConnectionState::Started
    }

    pub fn is_started(&self) -> bool {
        self.conn_state == ConnectionState::Started
    }

    pub async fn check_data(stream: &mut TcpStream) -> std::io::Result<bool> {
        let mut peek_buf = [0u8; 1];

        // // Try to peek at the stream
        // match stream.peek(&mut peek_buf) {
        //     Ok(0) => Ok(false), // Connection closed
        //     Ok(_) => Ok(true),  // Data is available
        //     Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(false), // No data yet
        //     Err(e) => Err(e),   // Some other error occurred
        // }

        match stream.peek(&mut peek_buf).await {
            Ok(0) => Ok(false), // Connection closed
            Ok(_) => Ok(true),  // Data is available
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(false), // No data yet
            Err(e) => Err(e),   // Some other error occurred
        }
    }

    pub async fn is_data_available(&mut self) -> bool {
        if let Some(ref mut stream) = self.stream {
            Self::check_data(stream).await.unwrap_or(false)
        } else {
            false
        }
    }

    pub async fn negotiate_protocol_version(&mut self) -> Result<(), RTDEError> {
        let cmd = RTDECommand::RtdeRequestProtocolVersion;
        let version: u8 = RTDE_PROTOCOL_VERSION;
        let buffer: Vec<u8> = vec![0, version]; // First byte is null, second is version

        debug!("Negotiating protocol version {}", version);
        self.send_all(cmd as u32, buffer.clone()).await?;
        debug!("Done sending RTDE_REQUEST_PROTOCOL_VERSION");

        // Receive and process response
        match self.receive().await {
            Ok(_) => {
                debug!("Protocol version negotiation successful");
                Ok(())
            },
            Err(e) => {
                debug!("Protocol version negotiation failed: {:?}", e);
                Err(e)
            },
        }
    }

    pub async fn receive(&mut self) -> Result<(), RTDEError> {
        debug!("Receiving...");

        // Read Header
        let mut header_data = vec![0u8; HEADER_SIZE as usize];
        let read_result = self
            .stream
            .as_mut()
            .ok_or(RTDEError::ConnectionError("No stream available".to_string()))?
            .read_exact(&mut header_data)
            .await;

        match read_result {
            Ok(_) => {
                let msg_size: u16 = u16::from_be_bytes([header_data[0], header_data[1]]);
                let msg_cmd: u8 = header_data[2];

                debug!("ControlHeader:");
                debug!("size is: {}", msg_size);
                debug!("command is: {}", msg_cmd);

                // Validate message size to prevent potential issues
                if msg_size < HEADER_SIZE || msg_size > 4096 {
                    return Err(RTDEError::ProtocolError(format!(
                        "Invalid message size: {}",
                        msg_size
                    )));
                }

                // Read Body
                let body_size = (msg_size - HEADER_SIZE) as usize;
                let mut data = vec![0u8; body_size];

                match self.stream.as_mut().unwrap().read_exact(&mut data).await {
                    Ok(_) => {
                        debug!("Received body data of size: {}", body_size);
                        // Process the response based on msg_cmd
                        match msg_cmd as u32 {
                            cmd if cmd == RTDECommand::RtdeTextMessage as u32 => {
                                let msg_length = data[0];
                                let msg_content =
                                    String::from_utf8_lossy(&data[1..msg_length as usize])
                                        .to_string();
                                debug!("Text message: {}", msg_content);
                            },
                            cmd if cmd == RTDECommand::RtdeRequestProtocolVersion as u32 => {
                                if body_size > 0 {
                                    debug!("Protocol version response: {:?}", data);
                                }
                            },
                            cmd if cmd == RTDECommand::RtdeGetUrcontrolVersion as u32 => {
                                debug!("ControlVersion:");
                                // Commented out as in original:
                                // let mut message_offset = 0;
                                // let v_major = get_uint32(&data, &mut message_offset);
                                // let v_minor = get_uint32(&data, &mut message_offset);
                                // let v_bugfix = get_uint32(&data, &mut message_offset);
                                // let v_build = get_uint32(&data, &mut message_offset);
                                // debug!("{}.{}.{}.{}", v_major, v_minor, v_bugfix, v_build);
                            },
                            cmd if cmd == RTDECommand::RtdeControlPackageSetupInputs as u32 => {
                                let datatypes = String::from_utf8_lossy(&data[1..]).to_string();
                                debug!("Datatype: {}", datatypes);

                                if datatypes.contains("IN_USE") {
                                    return Err(RTDEError::ProtocolError(
                                    "One of the RTDE input registers are already in use! Currently you must disable the EtherNet/IP adapter, \
                                     PROFINET or any MODBUS unit configured on the robot. This might change in the future.".to_string()
                                ));
                                }
                            },
                            cmd if cmd == RTDECommand::RtdeControlPackageSetupOutputs as u32 => {
                                let datatypes = String::from_utf8_lossy(&data[1..]).to_string();
                                debug!("Datatype: {}", datatypes);
                                self.output_types =
                                    datatypes.split(',').map(String::from).collect();

                                if datatypes.contains("NOT_FOUND") {
                                    let not_found_indexes: Vec<usize> = self
                                        .output_types
                                        .iter()
                                        .enumerate()
                                        .filter(|(_, t)| *t == "NOT_FOUND")
                                        .map(|(i, _)| i)
                                        .collect();

                                    let vars_not_found: String = not_found_indexes
                                        .iter()
                                        .enumerate()
                                        .map(|(i, &idx)| {
                                            let var = &self.output_names[idx];
                                            if i == not_found_indexes.len() - 1 {
                                                var.to_string()
                                            } else {
                                                format!("{}, ", var)
                                            }
                                        })
                                        .collect();

                                    let error_msg = format!(
                                    "The following variables was not found by the controller: [{}],\n \
                                     see which variables are supported by your PolyScope version here: \n\
                                     https://www.universal-robots.com/articles/ur/interface-communication/real-time-data-exchange-rtde-guide/",
                                    vars_not_found
                                );
                                    return Err(RTDEError::ProtocolError(error_msg));
                                }
                            },
                            cmd if cmd == RTDECommand::RtdeControlPackageStart as u32 => {
                                let success = data[0] != 0;
                                debug!("success: {}", success);

                                if success {
                                    self.conn_state = ConnectionState::Started;
                                    if self.verbose {
                                        println!("RTDE synchronization started");
                                    }
                                } else {
                                    eprintln!("Unable to start synchronization");
                                }
                            },
                            cmd if cmd == RTDECommand::RtdeControlPackagePause as u32 => {
                                let success = data[0] != 0;
                                debug!("success: {}", success);

                                if success {
                                    self.conn_state = ConnectionState::Connected; // Note: Using Connected instead of Paused
                                    debug!("RTDE synchronization paused!");
                                } else {
                                    eprintln!("Unable to pause synchronization");
                                }
                            },
                            _ => {
                                debug!("Unknown Command: {}", msg_cmd);
                            },
                        }
                        Ok(())
                    },
                    Err(e) => Err(RTDEError::ConnectionError(format!(
                        "Failed to read message body: {}",
                        e
                    ))),
                }
            },
            Err(e) => Err(RTDEError::ConnectionError(format!("Failed to read header: {}", e))),
        }
    }

    pub async fn receive_data(
        &mut self,
        robot_state: &Arc<Mutex<RobotState>>,
    ) -> Result<(), RTDEError> {
        debug!("Receiving data...");
        let mut message_offset: u32;
        let mut packet_data_offset: u32;

        // Prepare buffer for reading
        let mut data = vec![0u8; 4096];

        // Read with timeout
        let data_len = self.stream.as_mut().unwrap().read(&mut data).await?;
        debug!("Data len: {}", data_len);
        self.buffer.extend(data[..data_len].iter());

        while self.buffer.len() >= HEADER_SIZE as usize {
            debug!("Buffer len: {}", self.buffer.len());
            message_offset = 0;

            // Read RTDEControlHeader
            let packet_header = read_rtde_header(&self.buffer, &mut message_offset);

            debug!("Packet header msg size: {}", packet_header.msg_size);
            if self.buffer.len() >= packet_header.msg_size as usize {
                debug!(
                    "Packet header msg size: {}, buffer len: {}",
                    packet_header.msg_size,
                    self.buffer.len()
                );
                // Read data package and adjust buffer
                let mut header_and_packet: Vec<u8> =
                    self.buffer.drain(..packet_header.msg_size as usize).collect();
                let packet: Vec<u8> = header_and_packet.drain(HEADER_SIZE as usize..).collect();

                // Check for consecutive data packages
                // if self.buffer.len() >= HEADER_SIZE as usize
                //     && packet_header.msg_cmd == RTDECommand::RtdeDataPackage as u8
                // {
                //     debug!("Reading header")
                //     let next_header = read_rtde_header(&self.buffer, &mut message_offset);
                //     if next_header.msg_cmd == RTDECommand::RtdeDataPackage as u8 {
                //         if self.verbose {
                //             println!("skipping package(1)");
                //         }
                //         continue;
                //     }
                // }

                if packet_header.msg_cmd == RTDECommand::RtdeDataPackage as u8 {
                    packet_data_offset = 0;
                    let _recipe_id = get_u8(&packet, &mut packet_data_offset);

                    // Lock the robot state for updates
                    let mut robot_state_lock = robot_state.lock().await;

                    // Read all variables specified by the user
                    let mut counter = 0;
                    for output_name in &self.output_names {
                        if let Some(entry) = robot_state_lock.state_data.get(output_name) {
                            match entry {
                                StateDataTypes::VectorDouble(_) => {
                                    let parsed_data: Vec<f64> = if output_name
                                        == "actual_tool_accelerometer"
                                        || output_name == "payload_cog"
                                        || output_name == "elbow_position"
                                        || output_name == "elbow_velocity"
                                    {
                                        unpack_vector3d(&packet, &mut packet_data_offset)
                                    } else {
                                        unpack_vector6d(&packet, &mut packet_data_offset)
                                    };
                                    // debug!(
                                    //     "{}. offset={} {} VectorDouble Parsed data: {:?}",
                                    //     counter, packet_data_offset, output_name, parsed_data
                                    // );
                                    robot_state_lock.state_data.insert(
                                        output_name.to_string(),
                                        StateDataTypes::VectorDouble(parsed_data),
                                    );
                                },
                                StateDataTypes::Double(_) => {
                                    let parsed_data = get_double(&packet, &mut packet_data_offset);
                                    // debug!(
                                    //     "{}. offset={} {} Double Parsed data: {:?}",
                                    //     counter, packet_data_offset, output_name, parsed_data
                                    // );
                                    robot_state_lock.state_data.insert(
                                        output_name.to_string(),
                                        StateDataTypes::Double(parsed_data),
                                    );
                                },
                                StateDataTypes::Int32(_) => {
                                    let parsed_data = get_i32(&packet, &mut packet_data_offset);
                                    // debug!(
                                    //     "{}. offset={} {} Int32 Parsed data: {:?}",
                                    //     counter, packet_data_offset, output_name, parsed_data
                                    // );
                                    robot_state_lock.state_data.insert(
                                        output_name.to_string(),
                                        StateDataTypes::Int32(parsed_data),
                                    );
                                },
                                StateDataTypes::Uint32(_) => {
                                    let parsed_data = get_u32(&packet, &mut packet_data_offset);
                                    // debug!(
                                    //     "{}. offset={} {} Uint32 Parsed data: {:?}",
                                    //     counter, packet_data_offset, output_name, parsed_data
                                    // );
                                    robot_state_lock.state_data.insert(
                                        output_name.to_string(),
                                        StateDataTypes::Uint32(parsed_data),
                                    );
                                },
                                StateDataTypes::Uint64(_) => {
                                    let parsed_data = get_u64(&packet, &mut packet_data_offset);
                                    // debug!(
                                    //     "{}. offset={} {} Uint64 Parsed data: {:?}",
                                    //     counter, packet_data_offset, output_name, parsed_data
                                    // );
                                    robot_state_lock.state_data.insert(
                                        output_name.to_string(),
                                        StateDataTypes::Uint64(parsed_data),
                                    );
                                },
                                StateDataTypes::VectorInt(_) => {
                                    let parsed_data =
                                        unpack_vector6_i32(&packet, &mut packet_data_offset);
                                    // debug!(
                                    //     "{}. offset={} {} VectorInt Parsed data: {:?}",
                                    //     counter, packet_data_offset, output_name, parsed_data
                                    // );
                                    robot_state_lock.state_data.insert(
                                        output_name.to_string(),
                                        StateDataTypes::VectorInt(parsed_data),
                                    );
                                },
                                StateDataTypes::Boolean(_) => {
                                    let parsed_data = get_bool(&packet, &mut packet_data_offset);
                                    // debug!(
                                    //     "{}. offset={} {} Boolean Parsed data: {:?}",
                                    //     counter, packet_data_offset, output_name, parsed_data
                                    // );
                                    robot_state_lock.state_data.insert(
                                        output_name.to_string(),
                                        StateDataTypes::Boolean(parsed_data),
                                    );
                                },
                            }
                            counter += 1;
                        } else {
                            debug!(
                                "Unknown variable name: {} please verify the output setup!",
                                output_name
                            );
                        }
                    }

                    debug!("Robot state lock: {}", robot_state_lock.get_first_state_received());
                    if !robot_state_lock.get_first_state_received() {
                        robot_state_lock.set_first_state_received(true);
                    }

                    drop(robot_state_lock);
                } else if self.verbose {
                    debug!("skipping package(2)");
                }
            } else {
                debug!("Breaking");
                break;
            }
        }

        Ok(())
    }

    pub fn receive_output_types(&self) -> Result<(), RTDEError> {
        todo!()
    }

    pub async fn send(&mut self, command: RobotCommand) -> Result<(), RTDEError> {
        // info!("Sending robot command: {:?}", command);

        let data_cmd = RTDECommand::RtdeDataPackage as u8;
        let mut cmd_packed: Vec<u8> = pack_int32(command.command_type as i32);
        // info!("Command packed: {:?}", cmd_packed);

        // print raw binary balues of each cmd

        if command.command_type == CommandType::FtRtdeInputEnable
            || command.command_type == CommandType::EnableExternalFtSensor
        {
            let ft_rtde_input_enable_packed: Vec<u8> =
                pack_int32(command.ft_rtde_input_enable.unwrap());
            cmd_packed.extend(ft_rtde_input_enable_packed);
        }

        if command.command_type == CommandType::FreedriveMode {
            let free_axes_packed: Vec<u8> = pack_vector_n_int32(command.free_axes.unwrap());
            cmd_packed.extend(free_axes_packed);
        }

        if command.command_type == CommandType::SetInputIntRegister {
            let reg_int_packed: Vec<u8> = pack_int32(command.reg_int_val.unwrap());
            cmd_packed.extend(reg_int_packed);
        }

        if command.command_type == CommandType::SetInputDoubleRegister {
            let reg_double_packed: Vec<u8> = pack_double(command.reg_double_val.unwrap());
            cmd_packed.extend(reg_double_packed);
        }

        if command.command_type == CommandType::Watchdog {
            let watchdog_packed: Vec<u8> = pack_int32(CommandType::NoCmd as i32);
            cmd_packed.extend(watchdog_packed);
        }

        if command.command_type == CommandType::ForceMode {
            let force_mode_type_packed: Vec<u8> = pack_int32(command.force_mode_type.unwrap());
            cmd_packed.extend(force_mode_type_packed);

            let sel_vector_packed: Vec<u8> = pack_vector_n_int32(command.selection_vector.unwrap());
            cmd_packed.extend(sel_vector_packed);
        }

        if command.command_type == CommandType::GetActualJointPositionsHistory {
            let actual_joint_positions_history_packed: Vec<u8> =
                pack_uint32(command.steps.unwrap());
            cmd_packed.extend(actual_joint_positions_history_packed);
        }

        // info!("Command packed: {:?}", cmd_packed);

        if command.val.is_some() {
            let vector_nd_packed: Vec<u8> = pack_vector_n_double(command.val.unwrap());
            cmd_packed.extend(vector_nd_packed);
        }

        if command.command_type == CommandType::MoveJ
            || command.command_type == CommandType::MoveJIK
            || command.command_type == CommandType::MoveL
            || command.command_type == CommandType::MoveLFK
            || command.command_type == CommandType::MovePath
            || command.command_type == CommandType::StopJ
            || command.command_type == CommandType::StopL
        {
            let async_packed: Vec<u8> = pack_int32(command.async_.unwrap());
            cmd_packed.extend(async_packed);
        }

        if command.command_type == CommandType::SetStdDigitalOut {
            cmd_packed.push(command.std_digital_out_mask.unwrap());
            cmd_packed.push(command.std_digital_out.unwrap());
        }

        if command.command_type == CommandType::SetConfDigitalOut {
            cmd_packed.push(command.configurable_digital_out_mask.unwrap());
            cmd_packed.push(command.configurable_digital_out.unwrap());
        }

        if command.command_type == CommandType::SetToolDigitalOut {
            cmd_packed.push(command.std_tool_out_mask.unwrap());
            cmd_packed.push(command.std_tool_out.unwrap());
        }

        if command.command_type == CommandType::SetSpeedSlider {
            let speed_slider_mask_packed: Vec<u8> = pack_int32(command.speed_slider_mask.unwrap());
            cmd_packed.extend(speed_slider_mask_packed);

            let speed_slider_fraction_packed: Vec<u8> =
                pack_double(command.speed_slider_fraction.unwrap());
            cmd_packed.extend(speed_slider_fraction_packed);
        }

        if command.command_type == CommandType::SetStdAnalogOut {
            cmd_packed.push(command.std_analog_output_mask.unwrap());
            cmd_packed.push(command.std_analog_output_type.unwrap());

            let std_analog_output_0_packed: Vec<u8> =
                pack_double(command.std_analog_output_0.unwrap());
            cmd_packed.extend(std_analog_output_0_packed);

            let std_analog_output_1_packed: Vec<u8> =
                pack_double(command.std_analog_output_1.unwrap());
            cmd_packed.extend(std_analog_output_1_packed);
        }

        // info!("Command packed: {:?}", cmd_packed);

        cmd_packed.insert(0, command.recipe as u8);
        // info!("Command packed: {:?}", cmd_packed);

        // info!(
        //     "Command packed raw: {:?}",
        //     cmd_packed.iter().map(|b| format!("{:08b} ", b)).collect::<Vec<String>>()
        // );

        // print bytes of sent
        // info!("sent: {:?}", cmd_packed);

        self.send_all(data_cmd as u32, cmd_packed.clone()).await?;
        // debug!("Done sending RTDE_DATA_PACKAGE");
        self.receive().await?;
        Ok(())
    }

    pub async fn send_all(&mut self, command: u32, payload: Vec<u8>) -> Result<(), RTDEError> {
        // info!("Sending... {}", command);
        // payload.as_bytes().iter().for_each(|b| debug!("Byte: {}", b));
        // info!("Payload size: {}", payload.len());
        // info!("Payload string: {:?}", payload);

        let size: u16 = HEADER_SIZE + (payload.len() as u16);
        let cmd_type: u8 = command as u8;

        let mut header = [0u8; 3];
        header[0] = ((size >> 8) & 0xFF) as u8;
        header[1] = (size & 0xFF) as u8;
        header[2] = cmd_type;

        // info!("Header data: {} {} {}", size, size.to_be(), cmd_type);
        // info!("Header: {:?}", header);

        let mut buffer = Vec::with_capacity(header.len() + payload.len());
        buffer.extend_from_slice(&header);
        buffer.extend_from_slice(&payload);

        // info!("Buffer size: {}", buffer.len());
        // info!("Buffer: {:?}", buffer);
        // info!(
        //     "Buffer as string: {:?}",
        //     buffer.iter().map(|b| format!("{:08b} ", b)).collect::<Vec<String>>()
        // );

        if self.is_connected() {
            self.stream.as_mut().unwrap().write_all(&buffer).await?;
            // info!("Done sending all");
        }

        Ok(())
    }

    pub async fn send_start(&mut self) -> Result<(), RTDEError> {
        debug!("Sending start...");
        let cmd = RTDECommand::RtdeControlPackageStart;
        self.send_all(cmd as u32, Vec::new()).await?;
        debug!("Done sending RTDE_CONTROL_PACKAGE_START");
        self.receive().await?;
        Ok(())
    }

    pub async fn send_pause(&mut self) -> Result<(), RTDEError> {
        debug!("Sending pause...");
        let cmd = RTDECommand::RtdeControlPackagePause;
        self.send_all(cmd as u32, Vec::new()).await?;
        debug!("Done sending RTDE_CONTROL_PACKAGE_PAUSE");
        self.receive().await?;
        Ok(())
    }

    pub async fn send_clear(&mut self) -> Result<(), RTDEError> {
        let cmd = RobotCommand::new(CommandType::NoCmd, Recipe::Recipe4);
        self.send(cmd).await?;
        debug!("Done sending RTDE_CONTROL_PACKAGE_CLEAR");
        // self.receive().await?;
        Ok(())
    }

    // pub async fn stop_script(&mut self) -> Result<(), RTDEError> {
    //     let cmd = RobotCommand::new(CommandType::StopScript, Recipe::Recipe4);
    //     self.send_command(cmd).await?;
    //     debug!("Done sending RTDE_CONTROL_PACKAGE_STOP_SCRIPT");
    //     // self.receive().await?;
    //     Ok(())
    // }

    pub async fn send_output_setup(
        &mut self,
        output_names: &Vec<&str>,
        frequency: f64,
    ) -> Result<(), RTDEError> {
        let cmd = RTDECommand::RtdeControlPackageSetupOutputs;
        self.output_names = output_names.iter().map(|&s| s.to_string()).collect();
        let freq_as_hexstr = double2hexstr(frequency);
        let freq_packed = hex2bytes(&freq_as_hexstr);

        let output_names_str = self.output_names.join(",") + ",";

        debug!("freq_as_hexstr: {}", freq_as_hexstr);
        debug!("freq_packed: {:?}", freq_packed);
        debug!("output_names_str: {}", output_names_str);
        debug!("output setup lengths: {} {}", output_names_str.chars().count(), freq_packed.len());

        let mut payload: Vec<u8> =
            Vec::with_capacity(output_names_str.as_bytes().len() + freq_packed.len());
        payload.extend_from_slice(&freq_packed);
        payload.extend_from_slice(output_names_str.as_bytes());

        debug!("Payload: {} {:?}", payload.len(), payload);

        self.send_all(cmd as u32, payload).await?;
        debug!("Done sending RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS");
        self.receive().await?;
        Ok(())
    }

    pub async fn send_input_setup(&mut self, input_types: &Vec<&str>) -> Result<(), RTDEError> {
        let cmd: u32 = RTDECommand::RtdeControlPackageSetupInputs as u32;
        let payload = input_types.join(",") + ",";
        self.send_all(cmd, payload.as_bytes().to_vec()).await?;
        debug!("Done sending RTDE_CONTROL_PACKAGE_SETUP_INPUTS");
        self.receive().await?;
        Ok(())
    }
}
