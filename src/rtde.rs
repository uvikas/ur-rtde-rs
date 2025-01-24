/* RTDE Protocol */

use crate::robot_state::RobotState;
use log::debug;
use socket2::Socket;
use std::net::SocketAddr;
use std::net::TcpStream;
use tokio::time::Instant;

use crate::utils::{double2hexstr, hex2bytes};
const RTDE_PROTOCOL_VERSION: u8 = 2;

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
    buffer: Vec<char>,
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
        let sock = Socket::from(stream.try_clone()?);
        sock.set_reuse_address(true)?;
        self.stream = Some(stream);
        self.conn_state = ConnectionState::Connected;

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

    pub fn negotiate_protocol_version(&self) -> Result<(), RTDEError> {
        let cmd = RTDECommand::RtdeRequestProtocolVersion;
        let null_byte: u8 = 0;
        let version: u8 = RTDE_PROTOCOL_VERSION;
        let buffer: Vec<u8> = vec![null_byte, version];
        let payload: String = buffer.iter().map(|c| *c as char).collect();
        self.send_all(cmd as u32, payload)?;
        debug!("Done sending RTDE_REQUEST_PROTOCOL_VERSION");
        self.receive()?;
        Ok(())
    }

    pub fn receive(&self) -> Result<(), RTDEError> {
        todo!()
    }

    pub fn receive_data(&self, robot_state: &mut RobotState) -> Result<(), RTDEError> {
        // TODO: this is a big one
        todo!()
    }

    pub fn receive_output_types(&self) -> Result<(), RTDEError> {
        todo!()
    }

    pub fn send(&self, command: RobotCommand) -> Result<(), RTDEError> {
        todo!()
    }

    pub fn send_all(&self, command: u32, payload: String) -> Result<(), RTDEError> {
        todo!()
    }

    pub fn send_start(&self) -> Result<(), RTDEError> {
        let cmd = RTDECommand::RtdeControlPackageStart;
        self.send_all(cmd as u32, String::new())?;
        debug!("Done sending RTDE_CONTROL_PACKAGE_START");
        self.receive()?;
        Ok(())
    }

    pub fn send_pause(&self) -> Result<(), RTDEError> {
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
        let freq_packed = hex2bytes(freq_as_hexstr);
        let output_names_str = self.output_names.join(",");
        let payload = output_names_str + &freq_packed;
        self.send_all(cmd as u32, payload)?;
        debug!("Done sending RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS");
        self.receive()?;
        Ok(())
    }

    pub fn send_input_setup(&self, input_types: Vec<String>) -> Result<(), RTDEError> {
        todo!()
    }
}
