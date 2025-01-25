/* RTDE Protocol */

use crate::robot_state::{RobotState, StateDataTypes};
use log::debug;
use std::io::{Read, Write};
use std::net::SocketAddr;
use std::net::TcpStream;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::time::Instant;

use crate::utils::{
    double2hexstr, get_bool, get_double, get_i32, get_u32, get_u64, get_u8, hex2bytes,
    read_rtde_header, unpack_vector3d, unpack_vector6_i32, unpack_vector6d,
};

const RTDE_PROTOCOL_VERSION: u8 = 2;
const HEADER_SIZE: u16 = 3;

#[derive(Debug)]
#[allow(dead_code)]
enum CommandType {
    MoveJ,
    MoveJIK,
    MoveL,
    MoveLFK,
    ForceMode,
    ForceModeStop,
    ZeroFtSensor,
    SpeedJ,
    SpeedL,
    ServoJ,
    ServoC,
    SetStdDigitalOut,
    SetToolDigitalOut,
    SpeedStop,
    ServoStop,
    SetPayload,
    TeachMode,
    EndTeachMode,
    ForceModeSetDamping,
    ForceModeSetGainScaling,
    SetSpeedSlider,
    SetStdAnalogOut,
    ServoL,
    ToolContact,
    GetSteptime,
    GetActualJointPositionsHistory,
    GetTargetWaypoint,
    SetTcp,
    GetInverseKinematicsArgs,
    ProtectiveStop,
    StopL,
    StopJ,
    SetWatchdog,
    IsPoseWithinSafetyLimits,
    IsJointsWithinSafetyLimits,
    GetJointTorques,
    PoseTrans,
    GetTcpOffset,
    JogStart,
    JogStop,
    GetForwardKinematicsDefault,
    GetForwardKinematicsArgs,
    MovePath,
    GetInverseKinematicsDefault,
    IsSteady,
    SetConfDigitalOut,
    SetInputIntRegister,
    SetInputDoubleRegister,
    MoveUntilContact,
    FreedriveMode,
    EndFreedriveMode,
    GetFreedriveStatus,
    SetExternalForceTorque,
    FtRtdeInputEnable,
    EnableExternalFtSensor,
    GetActualToolFlangePose,
    SetGravity,
    GetInverseKinematicsHasSolutionDefault,
    GetInverseKinematicsHasSolutionArgs,
    StartContactDetection,
    StopContactDetection,
    ReadContactDetection,
    SetTargetPayload,
    StopScript,
    NoCmd,
}

#[derive(Debug)]
#[repr(u8)]
#[allow(dead_code)]
enum Recipe {
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

#[allow(dead_code)]
pub struct RobotCommand {
    command_type: CommandType,
    recipe: Recipe,
    ft_rtde_input_enable: Option<u32>,
    reg_int_val_: Option<i32>,
    reg_double_val_: Option<f64>,
    val_: Option<Vec<f64>>,
    selection_vector_: Option<Vec<i32>>,
    free_axes_: Option<Vec<i32>>,
    force_mode_type_: Option<i32>,
    std_digital_out_: Option<u8>,
    std_digital_out_mask_: Option<u8>,
    configurable_digital_out_: Option<u8>,
    configurable_digital_out_mask_: Option<u8>,
    std_tool_out_: Option<u8>,
    std_tool_out_mask_: Option<u8>,
    std_analog_output_mask_: Option<u8>,
    std_analog_output_type_: Option<u8>,
    std_analog_output_0_: Option<f64>,
    std_analog_output_1_: Option<f64>,
    speed_slider_mask_: Option<i32>,
    speed_slider_fraction_: Option<f64>,
    steps_: Option<u32>,
}

impl RobotCommand {
    pub fn new() -> Self {
        Self {
            command_type: CommandType::NoCmd,
            recipe: Recipe::Recipe1,
            ft_rtde_input_enable: None,
            reg_int_val_: None,
            reg_double_val_: None,
            val_: None,
            selection_vector_: None,
            free_axes_: None,
            force_mode_type_: None,
            std_digital_out_: None,
            std_digital_out_mask_: None,
            configurable_digital_out_: None,
            configurable_digital_out_mask_: None,
            std_tool_out_: None,
            std_tool_out_mask_: None,
            std_analog_output_mask_: None,
            std_analog_output_type_: None,
            std_analog_output_0_: None,
            std_analog_output_1_: None,
            speed_slider_mask_: None,
            speed_slider_fraction_: None,
            steps_: None,
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
}

impl std::error::Error for RTDEError {}

impl std::fmt::Display for RTDEError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            Self::ConnectionError(msg) => write!(f, "Connection error: {}", msg),
            Self::ProtocolError(msg) => write!(f, "Protocol error: {}", msg),
            Self::StateError(msg) => write!(f, "State error: {}", msg),
            Self::NoDataAvailable(msg) => write!(f, "No data available: {}", msg),
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
    stream: Option<std::net::TcpStream>,
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

    pub fn connect(&mut self) -> Result<(), RTDEError> {
        self.buffer.clear();

        let addr = format!("{}:{}", self.hostname, self.port)
            .parse::<SocketAddr>()
            .map_err(|e| format!("Invalid address: {}", e))?;

        let stream = std::net::TcpStream::connect(addr)?;
        stream.set_nodelay(true)?;

        // Create socket and configure it
        // let sock = Socket::from(stream.try_clone()?);
        // sock.set_reuse_address(true)?;
        self.stream = Some(stream);
        self.conn_state = ConnectionState::Connected;

        debug!("Connected to robot!");

        Ok(())
    }

    pub fn disconnect(&mut self, send_pause: bool) -> Result<(), RTDEError> {
        if self.conn_state == ConnectionState::Connected {
            if send_pause {
                self.send_pause()?;
            }
        }

        if let Some(stream) = self.stream.take() {
            stream.shutdown(std::net::Shutdown::Both)?;
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

    pub fn check_data(stream: &mut TcpStream) -> std::io::Result<bool> {
        let mut peek_buf = [0u8; 1];

        // Try to peek at the stream
        match stream.peek(&mut peek_buf) {
            Ok(0) => Ok(false), // Connection closed
            Ok(_) => Ok(true),  // Data is available
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(false), // No data yet
            Err(e) => Err(e),   // Some other error occurred
        }
    }

    pub fn is_data_available(&mut self) -> bool {
        if let Some(ref mut stream) = self.stream {
            Self::check_data(stream).unwrap_or(false)
        } else {
            false
        }
    }

    pub fn negotiate_protocol_version(&mut self) -> Result<(), RTDEError> {
        let cmd = RTDECommand::RtdeRequestProtocolVersion;
        let version: u8 = RTDE_PROTOCOL_VERSION;
        let buffer: Vec<u8> = vec![0, version]; // First byte is null, second is version

        debug!("Negotiating protocol version {}", version);
        let payload: String = String::from_utf8_lossy(&buffer).to_string();
        self.send_all(cmd as u32, payload)?;
        debug!("Done sending RTDE_REQUEST_PROTOCOL_VERSION");

        // Receive and process response
        match self.receive() {
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

    pub fn receive(&mut self) -> Result<(), RTDEError> {
        debug!("Receiving...");

        // Set read timeout on the socket
        self.stream
            .as_mut()
            .ok_or(RTDEError::ConnectionError("No stream available".to_string()))?
            .set_read_timeout(Some(Duration::from_secs(2)))?;

        // Read Header
        let mut header_data = vec![0u8; HEADER_SIZE as usize];
        let read_result = self
            .stream
            .as_mut()
            .ok_or(RTDEError::ConnectionError("No stream available".to_string()))?
            .read_exact(&mut header_data);

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

                match self.stream.as_mut().unwrap().read_exact(&mut data) {
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

    pub fn receive_data(&mut self, robot_state: &Arc<Mutex<RobotState>>) -> Result<(), RTDEError> {
        let mut message_offset: u32 = 0;
        let mut packet_data_offset: u32 = 0;

        // Prepare buffer of 4096 bytes
        let mut data = vec![0u8; 4096];

        // Read from socket
        let data_len = self
            .stream
            .as_mut()
            .ok_or(RTDEError::ConnectionError("No stream available".to_string()))?
            .read(&mut data)?;

        // Add data to the buffer
        self.buffer.extend(data[..data_len].iter());

        while self.buffer.len() >= HEADER_SIZE as usize {
            message_offset = 0;

            // Read RTDEControlHeader
            let packet_header = read_rtde_header(&self.buffer, &mut message_offset);

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
                if self.buffer.len() >= HEADER_SIZE as usize
                    && packet_header.msg_cmd == RTDECommand::RtdeDataPackage as u8
                {
                    let next_header = read_rtde_header(&self.buffer, &mut message_offset);
                    if next_header.msg_cmd == RTDECommand::RtdeDataPackage as u8 {
                        if self.verbose {
                            println!("skipping package(1)");
                        }
                        continue;
                    }
                }

                if packet_header.msg_cmd == RTDECommand::RtdeDataPackage as u8 {
                    packet_data_offset = 0;
                    let _recipe_id = get_u8(&packet, &mut packet_data_offset);

                    // Lock the robot state for updates
                    let mut robot_state_lock = robot_state.lock().unwrap();

                    // Read all variables specified by the user
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
                                    debug!(
                                        "{} VectorDouble Parsed data: {:?}",
                                        output_name, parsed_data
                                    );
                                    robot_state_lock.state_data.insert(
                                        output_name.to_string(),
                                        StateDataTypes::VectorDouble(parsed_data),
                                    );
                                },
                                StateDataTypes::Double(_) => {
                                    let parsed_data = get_double(&packet, &mut packet_data_offset);
                                    debug!("{} Double Parsed data: {:?}", output_name, parsed_data);
                                    robot_state_lock.state_data.insert(
                                        output_name.to_string(),
                                        StateDataTypes::Double(parsed_data),
                                    );
                                },
                                StateDataTypes::Int32(_) => {
                                    let parsed_data = get_i32(&packet, &mut packet_data_offset);
                                    debug!("{} Int32 Parsed data: {:?}", output_name, parsed_data);
                                    robot_state_lock.state_data.insert(
                                        output_name.to_string(),
                                        StateDataTypes::Int32(parsed_data),
                                    );
                                },
                                StateDataTypes::Uint32(_) => {
                                    let parsed_data = get_u32(&packet, &mut packet_data_offset);
                                    debug!("{} Uint32 Parsed data: {:?}", output_name, parsed_data);
                                    robot_state_lock.state_data.insert(
                                        output_name.to_string(),
                                        StateDataTypes::Uint32(parsed_data),
                                    );
                                },
                                StateDataTypes::Uint64(_) => {
                                    let parsed_data = get_u64(&packet, &mut packet_data_offset);
                                    debug!("{} Uint64 Parsed data: {:?}", output_name, parsed_data);
                                    robot_state_lock.state_data.insert(
                                        output_name.to_string(),
                                        StateDataTypes::Uint64(parsed_data),
                                    );
                                },
                                StateDataTypes::VectorInt(_) => {
                                    let parsed_data =
                                        unpack_vector6_i32(&packet, &mut packet_data_offset);
                                    debug!(
                                        "{} VectorInt Parsed data: {:?}",
                                        output_name, parsed_data
                                    );
                                    robot_state_lock.state_data.insert(
                                        output_name.to_string(),
                                        StateDataTypes::VectorInt(parsed_data),
                                    );
                                },
                                StateDataTypes::Boolean(_) => {
                                    let parsed_data = get_bool(&packet, &mut packet_data_offset);
                                    debug!(
                                        "{} Boolean Parsed data: {:?}",
                                        output_name, parsed_data
                                    );
                                    robot_state_lock.state_data.insert(
                                        output_name.to_string(),
                                        StateDataTypes::Boolean(parsed_data),
                                    );
                                },
                            }
                        } else {
                            debug!(
                                "Unknown variable name: {} please verify the output setup!",
                                output_name
                            );
                        }
                    }

                    if !robot_state_lock.get_first_state_received() {
                        robot_state_lock.set_first_state_received(true);
                    }

                    drop(robot_state_lock);
                } else if self.verbose {
                    println!("skipping package(2)");
                }
            } else {
                break;
            }
        }

        Ok(())
    }

    pub fn receive_output_types(&self) -> Result<(), RTDEError> {
        todo!()
    }

    pub fn send(&self, command: RobotCommand) -> Result<(), RTDEError> {
        todo!()
    }

    pub fn send_all(&mut self, command: u32, payload: String) -> Result<(), RTDEError> {
        debug!("Sending... {}", command);
        // payload.as_bytes().iter().for_each(|b| debug!("Byte: {}", b));
        debug!("Payload size: {}", payload.len());
        debug!("Payload string: {}", payload);

        let size: u16 = HEADER_SIZE + (payload.len() as u16);
        let cmd_type: u8 = command as u8;

        let mut header = [0u8; 3];
        header[0] = ((size >> 8) & 0xFF) as u8;
        header[1] = (size & 0xFF) as u8;
        header[2] = cmd_type;

        debug!("Header data: {} {} {}", size, size.to_be(), cmd_type);
        debug!("Header: {:?}", header);

        let payload_bytes = payload.as_bytes();
        let mut buffer = Vec::with_capacity(header.len() + payload_bytes.len());
        buffer.extend_from_slice(&header);
        buffer.extend_from_slice(payload_bytes);

        debug!("Buffer size: {}", buffer.len());
        debug!("Buffer: {:?}", buffer);
        debug!("Buffer as string: {}", String::from_utf8_lossy(&buffer));

        if self.is_connected() {
            self.stream.as_mut().unwrap().write_all(&buffer)?;
            debug!("Done sending all");
        }

        Ok(())
    }

    pub fn send_start(&mut self) -> Result<(), RTDEError> {
        let cmd = RTDECommand::RtdeControlPackageStart;
        self.send_all(cmd as u32, String::new())?;
        debug!("Done sending RTDE_CONTROL_PACKAGE_START");
        self.receive()?;
        Ok(())
    }

    pub fn send_pause(&mut self) -> Result<(), RTDEError> {
        let cmd = RTDECommand::RtdeControlPackagePause;
        self.send_all(cmd as u32, String::new())?;
        debug!("Done sending RTDE_CONTROL_PACKAGE_PAUSE");
        self.receive()?;
        Ok(())
    }

    pub fn send_output_setup(
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

        self.send_all(cmd as u32, String::from_utf8_lossy(&payload).to_string())?;
        debug!("Done sending RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS");
        self.receive()?;
        Ok(())
    }

    pub fn send_input_setup(&self, input_types: Vec<String>) -> Result<(), RTDEError> {
        todo!()
    }
}
