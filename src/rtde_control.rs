use crate::dashboard::DashboardClient;
use crate::robot_state::RobotState;
use crate::rtde::{
    CommandType, RTDEError, Recipe, RobotCommand, SafetyStatusBits, RTDE, RTDE_FIELDS,
    RTDE_START_SYNCHRONIZATION_TIMEOUT,
};
use crate::script_client::ScriptClient;
use log::{debug, error, info};
use std::error::Error;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use tokio::sync::Mutex;
use tokio::task::JoinHandle;
use tokio::time::{Duration, Instant};

const ASYNC_SETP_INPUT_RECIPE: &[&str] = &[
    "input_int_register_0",
    "input_double_register_0",
    "input_double_register_1",
    "input_double_register_2",
    "input_double_register_3",
    "input_double_register_4",
    "input_double_register_5",
    "input_double_register_6",
    "input_double_register_7",
    "input_int_register_1",
];

const SERVOJ_INPUT_RECIPE: &[&str] = &[
    "input_int_register_0",
    "input_double_register_0",
    "input_double_register_1",
    "input_double_register_2",
    "input_double_register_3",
    "input_double_register_4",
    "input_double_register_5",
    "input_double_register_6",
    "input_double_register_7",
    "input_double_register_8",
    "input_double_register_9",
    "input_double_register_10",
];

const FORCE_MODE_INPUT_RECIPE: &[&str] = &[
    "input_int_register_0",
    "input_int_register_1",
    "input_int_register_2",
    "input_int_register_3",
    "input_int_register_4",
    "input_int_register_5",
    "input_int_register_6",
    "input_int_register_7",
    "input_double_register_0",
    "input_double_register_1",
    "input_double_register_2",
    "input_double_register_3",
    "input_double_register_4",
    "input_double_register_5",
    "input_double_register_6",
    "input_double_register_7",
    "input_double_register_8",
    "input_double_register_9",
    "input_double_register_10",
    "input_double_register_11",
    "input_double_register_12",
    "input_double_register_13",
    "input_double_register_14",
    "input_double_register_15",
    "input_double_register_16",
    "input_double_register_17",
];

const NO_CMD_INPUT_RECIPE: &[&str] = &["input_int_register_0"];

const SERVO_C_INPUT_RECIPE: &[&str] = &[
    "input_int_register_0",
    "input_double_register_0",
    "input_double_register_1",
    "input_double_register_2",
    "input_double_register_3",
    "input_double_register_4",
    "input_double_register_5",
    "input_double_register_6",
    "input_double_register_7",
    "input_double_register_8",
];

const WRENCH_INPUT_RECIPE: &[&str] = &[
    "input_int_register_0",
    "input_double_register_0",
    "input_double_register_1",
    "input_double_register_2",
    "input_double_register_3",
    "input_double_register_4",
    "input_double_register_5",
];

const SET_PAYLOAD_INPUT_RECIPE: &[&str] = &[
    "input_int_register_0",
    "input_double_register_0",
    "input_double_register_1",
    "input_double_register_2",
    "input_double_register_3",
];

const FORCE_MODE_PARAMETERS_INPUT_RECIPE: &[&str] =
    &["input_int_register_0", "input_double_register_0"];

const GET_ACTUAL_JOINT_POSITIONS_HISTORY_INPUT_RECIPE: &[&str] =
    &["input_int_register_0", "input_int_register_1"];

const GET_INVERSE_KIN_INPUT_RECIPE: &[&str] = &[
    "input_int_register_0",
    "input_double_register_0",
    "input_double_register_1",
    "input_double_register_2",
    "input_double_register_3",
    "input_double_register_4",
    "input_double_register_5",
    "input_double_register_6",
    "input_double_register_7",
    "input_double_register_8",
    "input_double_register_9",
    "input_double_register_10",
    "input_double_register_11",
    "input_double_register_12",
    "input_double_register_13",
];

const WATCHDOG_INPUT_RECIPE: &[&str] = &["input_int_register_0"];

const POSE_TRANS_INPUT_RECIPE: &[&str] = &[
    "input_int_register_0",
    "input_double_register_0",
    "input_double_register_1",
    "input_double_register_2",
    "input_double_register_3",
    "input_double_register_4",
    "input_double_register_5",
    "input_double_register_6",
    "input_double_register_7",
    "input_double_register_8",
    "input_double_register_9",
    "input_double_register_10",
    "input_double_register_11",
];

const SETP_INPUT_RECIPE: &[&str] = &[
    "input_int_register_0",
    "input_double_register_0",
    "input_double_register_1",
    "input_double_register_2",
    "input_double_register_3",
    "input_double_register_4",
    "input_double_register_5",
    "input_double_register_6",
    "input_double_register_7",
];

const JOG_INPUT_RECIPE: &[&str] = &[
    "input_int_register_0",
    "input_double_register_0",
    "input_double_register_1",
    "input_double_register_2",
    "input_double_register_3",
    "input_double_register_4",
    "input_double_register_5",
    "input_double_register_6",
    "input_double_register_7",
    "input_double_register_8",
    "input_double_register_9",
    "input_double_register_10",
    "input_double_register_11",
    "input_double_register_12",
    "input_double_register_13",
];

const ASYNC_PATH_INPUT_RECIPE: &[&str] = &["input_int_register_0", "input_int_register_1"];

const MOVE_UNTIL_CONTACT_INPUT_RECIPE: &[&str] = &[
    "input_int_register_0",
    "input_double_register_0",
    "input_double_register_1",
    "input_double_register_2",
    "input_double_register_3",
    "input_double_register_4",
    "input_double_register_5",
    "input_double_register_6",
    "input_double_register_7",
    "input_double_register_8",
    "input_double_register_9",
    "input_double_register_10",
    "input_double_register_11",
    "input_double_register_12",
];

const FREEDRIVE_MODE_INPUT_RECIPE: &[&str] = &[
    "input_int_register_0",
    "input_int_register_1",
    "input_int_register_2",
    "input_int_register_3",
    "input_int_register_4",
    "input_int_register_5",
    "input_int_register_6",
    "input_double_register_0",
    "input_double_register_1",
    "input_double_register_2",
    "input_double_register_3",
    "input_double_register_4",
    "input_double_register_5",
];

const FT_RTDE_INPUT_ENABLE_RECIPE: &[&str] = &[
    "input_int_register_0",
    "input_int_register_1",
    "input_double_register_0",
    "input_double_register_1",
    "input_double_register_2",
    "input_double_register_3",
    "input_double_register_4",
    "input_double_register_5",
    "input_double_register_6",
];

const STOPL_STOPJ_INPUT_RECIPE: &[&str] =
    &["input_int_register_0", "input_double_register_0", "input_int_register_1"];

const SET_TARGET_PAYLOAD_INPUT_RECIPE: &[&str] = &[
    "input_int_register_0",
    "input_double_register_0",
    "input_double_register_1",
    "input_double_register_2",
    "input_double_register_3",
    "input_double_register_4",
    "input_double_register_5",
    "input_double_register_6",
    "input_double_register_7",
    "input_double_register_8",
    "input_double_register_9",
];

const EXTERNAL_FT_INPUT_RECIPE: &[&str] = &["input_int_register_0", "external_force_torque"];

const UR_CONTROLLER_RDY_FOR_CMD: u32 = 1;
const UR_CONTROLLER_DONE_WITH_CMD: u32 = 2;
const UR_EXECUTION_TIMEOUT: u32 = 300;

#[derive(Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum RuntimeState {
    Stopping = 0,
    Stopped = 1,
    Playing = 2,
    Pausing = 3,
    Paused = 4,
    Resuming = 5,
}

pub struct RTDEControl {
    hostname: String,
    dt: f64,
    fields: Vec<String>,

    rtde: Arc<Mutex<RTDE>>,
    robot_state: Arc<Mutex<RobotState>>,
    script_client: Arc<Mutex<ScriptClient>>,

    db_client: Arc<Mutex<DashboardClient>>,
    no_bytes_avail_cnt: Arc<Mutex<u32>>,
    stop_receive_thread: Arc<AtomicBool>,
    receive_thread: Option<JoinHandle<()>>,
}

impl RTDEControl {
    pub async fn new(hostname: &str) -> Result<Self, RTDEError> {
        let db_client = Arc::new(Mutex::new(DashboardClient::new(hostname)));

        {
            debug!("Connecting to dashboard");
            let mut db_client_lock = db_client.lock().await;
            db_client_lock.connect().await?;

            if !db_client_lock.is_in_remote_control().await? {
                return Err(RTDEError::RobotNotInRemoteControl(format!(
                    "Robot not in remote control: {}",
                    hostname
                )));
            }
        }

        // info!("Connected to dashboard");

        let frequency: f64 = 500.0;
        let rtde = Arc::new(Mutex::new(RTDE::new(hostname)));
        {
            debug!("Connecting to robot");
            let mut rtde_lock = rtde.lock().await;
            rtde_lock.connect().await?;
            rtde_lock.negotiate_protocol_version().await?;
        }

        // info!("Connected to robot");

        let script_client = Arc::new(Mutex::new(ScriptClient::new(hostname)));
        {
            let mut script_client_lock = script_client.lock().await;
            script_client_lock.connect().await?;
        }

        // info!("Connected to script client");

        let fields = vec![
            "robot_status_bits",
            "safety_status_bits",
            "runtime_state",
            "output_int_register_0",
            "output_int_register_1",
            "output_int_register_2",
            "output_double_register_0",
            "output_double_register_1",
            "output_double_register_2",
            "output_double_register_3",
            "output_double_register_4",
            "output_double_register_5",
        ];

        let robot_state = Arc::new(Mutex::new(RobotState::new(&fields)));

        {
            let mut rtde_lock = rtde.lock().await;
            rtde_lock.send_output_setup(&fields, frequency).await?;

            rtde_lock.send_input_setup(&ASYNC_SETP_INPUT_RECIPE.to_vec()).await?;
            rtde_lock.send_input_setup(&SERVOJ_INPUT_RECIPE.to_vec()).await?;
            rtde_lock.send_input_setup(&FORCE_MODE_INPUT_RECIPE.to_vec()).await?;
            rtde_lock.send_input_setup(&NO_CMD_INPUT_RECIPE.to_vec()).await?;
            rtde_lock.send_input_setup(&SERVO_C_INPUT_RECIPE.to_vec()).await?;
            rtde_lock.send_input_setup(&WRENCH_INPUT_RECIPE.to_vec()).await?;
            rtde_lock.send_input_setup(&SET_PAYLOAD_INPUT_RECIPE.to_vec()).await?;
            rtde_lock.send_input_setup(&FORCE_MODE_PARAMETERS_INPUT_RECIPE.to_vec()).await?;
            rtde_lock
                .send_input_setup(&GET_ACTUAL_JOINT_POSITIONS_HISTORY_INPUT_RECIPE.to_vec())
                .await?;
            rtde_lock.send_input_setup(&GET_INVERSE_KIN_INPUT_RECIPE.to_vec()).await?;
            rtde_lock.send_input_setup(&WATCHDOG_INPUT_RECIPE.to_vec()).await?;
            rtde_lock.send_input_setup(&POSE_TRANS_INPUT_RECIPE.to_vec()).await?;
            rtde_lock.send_input_setup(&SETP_INPUT_RECIPE.to_vec()).await?;
            rtde_lock.send_input_setup(&JOG_INPUT_RECIPE.to_vec()).await?;
            rtde_lock.send_input_setup(&ASYNC_PATH_INPUT_RECIPE.to_vec()).await?;
            rtde_lock.send_input_setup(&MOVE_UNTIL_CONTACT_INPUT_RECIPE.to_vec()).await?;
            rtde_lock.send_input_setup(&FREEDRIVE_MODE_INPUT_RECIPE.to_vec()).await?;
            rtde_lock.send_input_setup(&FT_RTDE_INPUT_ENABLE_RECIPE.to_vec()).await?;
            rtde_lock.send_input_setup(&STOPL_STOPJ_INPUT_RECIPE.to_vec()).await?;
            rtde_lock.send_input_setup(&SET_TARGET_PAYLOAD_INPUT_RECIPE.to_vec()).await?;
            rtde_lock.send_input_setup(&EXTERNAL_FT_INPUT_RECIPE.to_vec()).await?;
            rtde_lock.send_start().await?;

            let start_time = Instant::now();
            while !rtde_lock.is_started() {
                if start_time.elapsed() >= Duration::from_secs(RTDE_START_SYNCHRONIZATION_TIMEOUT) {
                    return Err(RTDEError::RobotConnectionTimeout(
                        "RTDE did not start within start synchronization timeout.".to_string(),
                    ));
                }
                tokio::time::sleep(Duration::from_millis(2)).await;
            }
        }

        // info!("RTDE started");

        let mut interface = Self {
            hostname: hostname.to_string(),
            dt: 1.0 / frequency,
            fields: fields.iter().map(|&s| s.to_string()).collect(),
            robot_state: robot_state.clone(),
            rtde: rtde.clone(),
            db_client: db_client.clone(),
            script_client: script_client.clone(),
            no_bytes_avail_cnt: Arc::new(Mutex::new(0)),
            stop_receive_thread: Arc::new(AtomicBool::new(false)),
            receive_thread: None,
        };

        let robot_state_guard = robot_state.lock().await;
        let rtde_ready = robot_state_guard.first_state_received.clone();
        drop(robot_state_guard);

        interface.start_receive_thread().await?;

        while !rtde_ready.load(Ordering::Relaxed) {
            debug!("Waiting for robot state...");
            tokio::time::sleep(Duration::from_millis(2)).await;
        }

        // info!("Robot state received");

        interface.send_clear_command().await?;

        // upload script
        {
            let mut script_client_lock = script_client.lock().await;
            let mut db_client_lock = db_client.lock().await;

            if !interface.is_program_running().await {
                // info!("Uploading script");
                script_client_lock.send_script().await?;
                interface.wait_for_program_running().await?;
            } else {
                // info!("Stopping script");
                interface.stop_script().await?;
                // info!("Stopping dashboard");
                db_client_lock.stop().await?;
                tokio::time::sleep(Duration::from_millis(100)).await;
                // info!("Uploading script");
                script_client_lock.send_script().await?;
                interface.wait_for_program_running().await?;
            }
        }

        // info!("Script uploaded");

        Ok(interface)
    }

    pub async fn start_receive_thread(&mut self) -> Result<(), RTDEError> {
        let rtde = self.rtde.clone();
        let robot_state = self.robot_state.clone();
        let stop_receive_thread = self.stop_receive_thread.clone();
        let no_bytes_avail_cnt = self.no_bytes_avail_cnt.clone();

        self.stop_receive_thread.store(false, Ordering::SeqCst);

        self.receive_thread = Some(tokio::spawn(async move {
            debug!("STARTED receive thread {}", stop_receive_thread.load(Ordering::Relaxed));
            while !stop_receive_thread.load(Ordering::Relaxed) {
                debug!("Executing receive thread");
                let is_available_future = {
                    let mut rtde_lock = rtde.lock().await;
                    rtde_lock.is_data_available().await
                };
                if is_available_future {
                    let mut no_bytes_avail_cnt_lock = no_bytes_avail_cnt.lock().await;
                    *no_bytes_avail_cnt_lock = 0;

                    {
                        let mut rtde_lock = rtde.lock().await;
                        rtde_lock.receive_data(&robot_state).await.unwrap();
                    }
                } else {
                    let mut no_bytes_avail_cnt_lock = no_bytes_avail_cnt.lock().await;
                    *no_bytes_avail_cnt_lock += 1;
                    if *no_bytes_avail_cnt_lock > 100 {
                        error!("No bytes available");
                    }
                }
                tokio::time::sleep(Duration::from_millis(2)).await;
            }
            debug!("STOPPED receive thread {}", stop_receive_thread.load(Ordering::Relaxed));
        }));

        Ok(())
    }

    pub async fn send_clear_command(&mut self) -> Result<(), RTDEError> {
        let mut rtde_lock = self.rtde.lock().await;
        rtde_lock.send_clear().await?;
        Ok(())
    }

    pub async fn stop_script(&mut self) -> Result<(), RTDEError> {
        let cmd = RobotCommand::new(CommandType::StopScript, Recipe::Recipe4);

        self.send_command(cmd).await?;

        Ok(())
    }

    pub async fn send_command(&mut self, command: RobotCommand) -> Result<(), RTDEError> {
        // info!("Sending command {:?}", command);
        let start_time = Instant::now();
        let command_type = command.command_type;

        let runtime_state: u32;
        {
            let robot_state_lock = self.robot_state.lock().await;
            runtime_state = robot_state_lock.get_state_data("runtime_state").unwrap().as_uint32();
        }

        if runtime_state == RuntimeState::Stopped as u32 {
            // info!("Runtime state is stopped");
            self.send_clear_command().await?;
            return Ok(());
        }

        if self.is_program_running().await {
            // info!("Program is running");
            while self.get_control_script_state().await != UR_CONTROLLER_RDY_FOR_CMD {
                // info!("Waiting for control script to be ready");
                if self.is_protective_stopped().await || self.is_emergency_stopped().await {
                    // info!("Protective or emergency stop");
                    self.send_clear_command().await?;
                    return Ok(());
                }
            }

            if command_type == CommandType::ServoJ
                || command_type == CommandType::ServoL
                || command_type == CommandType::ServoC
                || command_type == CommandType::SpeedJ
                || command_type == CommandType::SpeedL
                || command_type == CommandType::ForceMode
                || command_type == CommandType::Watchdog
                || command_type == CommandType::GetJointTorques
                || command_type == CommandType::ToolContact
                || command_type == CommandType::GetSteptime
                || command_type == CommandType::GetActualJointPositionsHistory
                || command_type == CommandType::SetExternalForceTorque
            {
                // info!("1 Sending command to robot");
                let mut rtde_lock = self.rtde.lock().await;
                rtde_lock.send(command).await?;
                return Ok(());
            } else {
                {
                    let mut rtde_lock = self.rtde.lock().await;
                    // info!("2 Sending command to robot {:?}", command);
                    rtde_lock.send(command).await?;
                }

                if command_type != CommandType::StopScript {
                    let start_time = Instant::now();
                    while self.get_control_script_state().await != UR_CONTROLLER_DONE_WITH_CMD {
                        if !self.is_program_running().await {
                            self.send_clear_command().await?;
                            return Err(RTDEError::RobotConnectionTimeout(
                                "RTDE control script unexpectedly stopped".to_string(),
                            ));
                        }

                        if self.is_protective_stopped().await || self.is_emergency_stopped().await {
                            self.send_clear_command().await?;
                            return Ok(());
                        }

                        let duration = start_time.elapsed().as_secs_f32();
                        if duration > UR_EXECUTION_TIMEOUT as f32 {
                            self.send_clear_command().await?;
                            return Err(RTDEError::RobotConnectionTimeout(
                                "RTDE control script timed out".to_string(),
                            ));
                        }

                        tokio::time::sleep(Duration::from_millis(2)).await;
                    }
                } else {
                    // info!("Waiting for program to stop");
                    while self.is_program_running().await {
                        if self.is_protective_stopped().await || self.is_emergency_stopped().await {
                            // info!("Protective or emergency stop");
                            self.send_clear_command().await?;
                            return Ok(());
                        }

                        let duration = start_time.elapsed().as_secs_f32();
                        if duration > UR_EXECUTION_TIMEOUT as f32 {
                            self.send_clear_command().await?;
                            return Err(RTDEError::RobotConnectionTimeout(
                                "RTDE control script timed out".to_string(),
                            ));
                        }

                        tokio::time::sleep(Duration::from_millis(2)).await;
                    }
                }
            }

            self.send_clear_command().await?;
            return Ok(());
        }

        self.send_clear_command().await?;
        return Err(RTDEError::RobotConnectionTimeout(
            "RTDE control script is not running".to_string(),
        ));
    }

    pub async fn get_control_script_state(&mut self) -> u32 {
        let out_int_reg_0 = self
            .robot_state
            .lock()
            .await
            .get_state_data("output_int_register_0")
            .unwrap()
            .as_int32();

        //  info!("Control script state: {}", out_int_reg_0);
        out_int_reg_0 as u32
    }

    pub async fn is_protective_stopped(&mut self) -> bool {
        let robot_state_lock = self.robot_state.lock().await;
        let safety_status_bits =
            robot_state_lock.get_state_data("safety_status_bits").unwrap().as_uint32();

        let is_protective_stopped: bool =
            safety_status_bits & (1 << SafetyStatusBits::IsProtectiveStopped as u32) != 0;

        is_protective_stopped
    }

    pub async fn is_emergency_stopped(&mut self) -> bool {
        let robot_state_lock = self.robot_state.lock().await;
        let safety_status_bits =
            robot_state_lock.get_state_data("safety_status_bits").unwrap().as_uint32();
        let is_emergency_stopped: bool =
            safety_status_bits & (1 << SafetyStatusBits::IsEmergencyStopped as u32) != 0;

        is_emergency_stopped
    }

    pub async fn is_program_running(&mut self) -> bool {
        let robot_state_lock = self.robot_state.lock().await;
        let runtime_state = robot_state_lock.get_state_data("runtime_state").unwrap().as_uint32();

        // info!("Runtime state: {}", runtime_state);

        runtime_state == RuntimeState::Playing as u32
    }

    pub async fn wait_for_program_running(&mut self) -> Result<(), RTDEError> {
        let mut ms_count = 0;
        let sleep_ms = 10;

        while !self.is_program_running().await {
            tokio::time::sleep(Duration::from_millis(sleep_ms)).await;
            ms_count += sleep_ms;
            if ms_count > 5000 {
                return Err(RTDEError::RobotConnectionTimeout(
                    "Failed to start control script, before timeout of 5 seconds".to_string(),
                ));
            }
        }
        Ok(())
    }

    pub async fn disconnect(&mut self) -> Result<(), RTDEError> {
        let mut rtde_lock = self.rtde.lock().await;
        rtde_lock.disconnect().await?;
        Ok(())
    }
}
