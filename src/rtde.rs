/* RTDE Protocol */

use log::debug;
use socket2::Socket;
use std::{error::Error, net::SocketAddr};
use tokio::time::Instant;

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
#[repr(u8)]

enum ConnectionState {
    Disconnected = 0,
    Connected = 1,
    Started = 2,
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
    pub fn new(hostname: String) -> Self {
        Self {
            hostname: hostname,
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

    pub fn connect(&mut self) -> Result<(), Box<dyn Error>> {
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

    pub fn disconnect(&mut self, send_pause: bool) -> Result<(), Box<dyn Error>> {
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

    pub fn is_data_available(&self) -> bool {
        todo!()
    }

    pub fn negogiate_protocol_version(&self) -> Result<(), Box<dyn Error>> {
        todo!()
    }

    pub fn get_controller_version(&self) -> Result<(), Box<dyn Error>> {
        todo!()
    }

    pub fn receive_data(&self) -> Result<(), Box<dyn Error>> {
        todo!()
    }

    pub fn receive_output_types(&self) -> Result<(), Box<dyn Error>> {
        todo!()
    }

    pub fn send(&self, command: RobotCommand) -> Result<(), Box<dyn Error>> {
        todo!()
    }

    pub fn send_all(&self, command: u8) -> Result<(), Box<dyn Error>> {
        todo!()
    }

    pub fn send_start(&self) -> Result<(), Box<dyn Error>> {
        todo!()
    }

    pub fn send_pause(&self) -> Result<(), Box<dyn Error>> {
        todo!()
    }

    pub fn send_output_setup(
        &self,
        output_types: Vec<String>,
        frequency: f64,
    ) -> Result<(), Box<dyn Error>> {
        todo!()
    }

    pub fn send_input_setup(&self, input_types: Vec<String>) -> Result<(), Box<dyn Error>> {
        todo!()
    }
}
