// RTDE Receive Interface for reading data from UR robot

use std::error::Error;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Duration;
use tokio::sync::Mutex;

use crate::robot_state::RobotState;
use crate::rtde::{RTDEError, RTDE};
use log::{debug, info};

// All RTDE fields for e-series UR
const RTDE_FIELDS: &[&str] = &[
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

#[repr(u32)]
enum SafetyStatusBits {
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

// RTDE Receive Interface
pub struct RTDEReceive {
    hostname: String,
    dt: f64,
    fields: Vec<String>,

    robot_state: Arc<Mutex<RobotState>>,
    rtde: Arc<Mutex<RTDE>>,

    stop_receive_thread: Arc<AtomicBool>,
    receive_thread: Option<tokio::task::JoinHandle<()>>,
    no_bytes_avail_cnt: Arc<Mutex<u32>>,
}

impl RTDEReceive {
    // Create new RTDE Receive Interface.
    pub async fn new(hostname: &str) -> Result<Self, RTDEError> {
        let frequency: f64 = 500.0;
        let rtde = Arc::new(Mutex::new(RTDE::new(hostname)));

        {
            debug!("Connecting to robot");
            let mut rtde_lock = rtde.lock().await;
            rtde_lock.connect().await?;
            rtde_lock.negotiate_protocol_version().await?;
        }

        let fields = RTDE_FIELDS.to_vec();
        debug!("fields: {:?}", fields);
        let robot_state = Arc::new(Mutex::new(RobotState::new(&fields)));

        {
            let mut rtde_lock = rtde.lock().await;
            rtde_lock.send_output_setup(&fields, frequency).await?;
            rtde_lock.send_start().await?;
        }

        let mut interface = Self {
            hostname: hostname.to_string(),
            dt: 1.0 / frequency,
            fields: fields.iter().map(|&s| s.to_string()).collect(),
            robot_state: robot_state.clone(),
            rtde: rtde.clone(),
            stop_receive_thread: Arc::new(AtomicBool::new(false)),
            receive_thread: None,
            no_bytes_avail_cnt: Arc::new(Mutex::new(0)),
        };

        let robot_state_guard = robot_state.lock().await;
        let rtde_ready = robot_state_guard.first_state_received.clone();
        drop(robot_state_guard);

        interface.start_receive_thread().await?;

        info!("Waiting for robot state...");

        while !rtde_ready.load(Ordering::Relaxed) {
            debug!("Waiting for robot state...");
            tokio::time::sleep(Duration::from_millis(2)).await;
        }

        info!("Robot state received");

        Ok(interface)
    }

    // Start receive thread.
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
                // if is_available_future {
                let mut no_bytes_avail_cnt_lock = no_bytes_avail_cnt.lock().await;
                *no_bytes_avail_cnt_lock = 0;

                {
                    let mut rtde_lock = rtde.lock().await;
                    rtde_lock.receive_data(&robot_state).await.unwrap();
                }
                // } else {
                //     let mut no_bytes_avail_cnt_lock = no_bytes_avail_cnt.lock().await;
                //     *no_bytes_avail_cnt_lock += 1;
                //     if *no_bytes_avail_cnt_lock > 20 {
                //         debug!("No bytes available");
                //     }
                // }
                tokio::time::sleep(Duration::from_millis(2)).await;
            }
            debug!("STOPPED receive thread {}", stop_receive_thread.load(Ordering::Relaxed));
        }));

        Ok(())
    }

    pub async fn get_actual_q(&self) -> Result<Vec<f64>, Box<dyn Error>> {
        let robot_state_lock = self.robot_state.lock().await;
        let actual_q = robot_state_lock.get_state_data("actual_q")?;
        Ok(actual_q.as_vec_double())
    }

    pub async fn get_actual_qd(&self) -> Result<Vec<f64>, Box<dyn Error>> {
        let robot_state_lock = self.robot_state.lock().await;
        let actual_qd = robot_state_lock.get_state_data("actual_qd")?;
        Ok(actual_qd.as_vec_double())
    }

    pub async fn get_actual_tcp_pose(&self) -> Result<Vec<f64>, Box<dyn Error>> {
        let robot_state_lock = self.robot_state.lock().await;
        let actual_tcp_pose = robot_state_lock.get_state_data("actual_TCP_pose")?;
        Ok(actual_tcp_pose.as_vec_double())
    }

    pub async fn get_actual_tcp_speed(&self) -> Result<Vec<f64>, Box<dyn Error>> {
        let robot_state_lock = self.robot_state.lock().await;
        let actual_tcp_speed = robot_state_lock.get_state_data("actual_TCP_speed")?;
        Ok(actual_tcp_speed.as_vec_double())
    }

    pub async fn get_actual_tcp_force(&self) -> Result<Vec<f64>, Box<dyn Error>> {
        let robot_state_lock = self.robot_state.lock().await;
        let actual_tcp_force = robot_state_lock.get_state_data("actual_TCP_force")?;
        Ok(actual_tcp_force.as_vec_double())
    }

    pub async fn get_robot_mode(&self) -> Result<u32, Box<dyn Error>> {
        let robot_state_lock = self.robot_state.lock().await;
        let robot_mode = robot_state_lock.get_state_data("robot_mode")?;
        Ok(robot_mode.as_uint32())
    }

    pub async fn is_protective_stopped(&self) -> Result<bool, Box<dyn Error>> {
        let robot_state_lock = self.robot_state.lock().await;
        let safety_status_bits = robot_state_lock.get_state_data("safety_status_bits")?;
        let is_protective_stopped: bool = safety_status_bits.as_uint32()
            & (1 << SafetyStatusBits::IsProtectiveStopped as u32)
            != 0;
        Ok(is_protective_stopped)
    }

    pub async fn is_connected(&self) -> bool {
        let rtde_lock = self.rtde.lock().await;
        rtde_lock.is_connected()
    }

    pub async fn disconnect(&mut self) -> Result<(), RTDEError> {
        self.stop_receive_thread.store(true, Ordering::Relaxed);
        if let Some(thread) = self.receive_thread.take() {
            thread.await.unwrap();
        }
        let mut rtde_lock = self.rtde.lock().await;
        if rtde_lock.is_connected() {
            rtde_lock.disconnect().await?;
        }
        Ok(())
    }
}
