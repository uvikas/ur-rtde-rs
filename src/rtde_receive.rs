// RTDE Receive Interface for reading data from UR robot

use std::error::Error;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use crate::robot_state::RobotState;
use crate::rtde::{RTDEError, RTDE};

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
    stop_record_thread: Arc<AtomicBool>,
    receive_thread: Option<thread::JoinHandle<()>>,
    no_bytes_avail_cnt: u32,
}

impl RTDEReceive {
    // Create new RTDE Receive Interface.
    pub fn new(hostname: &str) -> Result<Self, RTDEError> {
        let frequency: f64 = 500.0;
        let rtde = Arc::new(Mutex::new(RTDE::new(hostname)));

        {
            let mut rtde_lock = rtde.lock().unwrap();
            rtde_lock.connect()?;
            rtde_lock.negotiate_protocol_version()?;
        }

        let fields = RTDE_FIELDS.to_vec();
        let robot_state = Arc::new(Mutex::new(RobotState::new(&fields)));

        {
            let mut rtde_lock = rtde.lock().unwrap();
            rtde_lock.send_output_setup(&fields, frequency)?;
            rtde_lock.send_start()?;
        }

        let mut interface = Self {
            hostname: hostname.to_string(),
            dt: 1.0 / frequency,
            fields: fields.iter().map(|&s| s.to_string()).collect(),
            robot_state: robot_state.clone(),
            rtde: rtde.clone(),
            stop_receive_thread: Arc::new(AtomicBool::new(false)),
            stop_record_thread: Arc::new(AtomicBool::new(false)),
            receive_thread: None,
            no_bytes_avail_cnt: 0,
        };
        interface.start_receive_thread()?;

        while {
            let rtde_lock = rtde.lock().unwrap();
            let robot_state_lock = robot_state.lock().unwrap();
            rtde_lock.is_connected() && !robot_state_lock.get_first_state_received()
        } {
            thread::sleep(Duration::from_millis(100));
        }

        Ok(interface)
    }

    // Start receive thread.
    pub fn start_receive_thread(&mut self) -> Result<(), RTDEError> {
        let rtde = self.rtde.clone();
        let robot_state = self.robot_state.clone();
        let stop_receive_thread = self.stop_receive_thread.clone();

        self.receive_thread = Some(thread::spawn(move || {
            while !stop_receive_thread.load(Ordering::Relaxed) {
                // Use cloned values instead of self
                let mut rtde_lock = rtde.lock().unwrap();
                let mut robot_state_lock = robot_state.lock().unwrap();

                if rtde_lock.is_data_available() {
                    rtde_lock.receive_data(&mut robot_state_lock).unwrap();
                }
                thread::sleep(Duration::from_millis(100));
            }
        }));

        Ok(())
    }

    fn get_available_bytes(&mut self) -> Result<(), RTDEError> {
        let mut rtde_lock = self.rtde.lock().unwrap();
        let mut robot_state_lock = self.robot_state.lock().unwrap();

        if rtde_lock.is_data_available() {
            self.no_bytes_avail_cnt = 0;
            rtde_lock.receive_data(&mut robot_state_lock)?;
            return Ok(());
        } else {
            self.no_bytes_avail_cnt += 1;
            if self.no_bytes_avail_cnt > 20 {
                match rtde_lock.receive_data(&mut robot_state_lock) {
                    Ok(_) => {
                        self.no_bytes_avail_cnt = 0;
                        return Ok(());
                    },
                    Err(e) => {
                        return Err(RTDEError::NoDataAvailable(e.to_string()));
                    },
                }
            }
        }
        Ok(())
    }

    pub fn get_actual_q(&self) -> Result<Vec<f64>, Box<dyn Error>> {
        let robot_state_lock = self.robot_state.lock().unwrap();
        let actual_q = robot_state_lock.get_state_data("actual_q")?;
        Ok(actual_q.as_vec_double())
    }

    pub fn get_actual_qd(&self) -> Result<Vec<f64>, Box<dyn Error>> {
        let robot_state_lock = self.robot_state.lock().unwrap();
        let actual_qd = robot_state_lock.get_state_data("actual_qd")?;
        Ok(actual_qd.as_vec_double())
    }

    pub fn get_actual_tcp_pose(&self) -> Result<Vec<f64>, Box<dyn Error>> {
        let robot_state_lock = self.robot_state.lock().unwrap();
        let actual_tcp_pose = robot_state_lock.get_state_data("actual_TCP_pose")?;
        Ok(actual_tcp_pose.as_vec_double())
    }

    pub fn get_actual_tcp_speed(&self) -> Result<Vec<f64>, Box<dyn Error>> {
        let robot_state_lock = self.robot_state.lock().unwrap();
        let actual_tcp_speed = robot_state_lock.get_state_data("actual_TCP_speed")?;
        Ok(actual_tcp_speed.as_vec_double())
    }

    pub fn get_actual_tcp_force(&self) -> Result<Vec<f64>, Box<dyn Error>> {
        let robot_state_lock = self.robot_state.lock().unwrap();
        let actual_tcp_force = robot_state_lock.get_state_data("actual_TCP_force")?;
        Ok(actual_tcp_force.as_vec_double())
    }

    pub fn get_robot_mode(&self) -> Result<u32, Box<dyn Error>> {
        let robot_state_lock = self.robot_state.lock().unwrap();
        let robot_mode = robot_state_lock.get_state_data("robot_mode")?;
        Ok(robot_mode.as_uint32())
    }

    pub fn is_protective_stopped(&self) -> Result<bool, Box<dyn Error>> {
        let robot_state_lock = self.robot_state.lock().unwrap();
        let safety_status_bits = robot_state_lock.get_state_data("safety_status_bits")?;
        let is_protective_stopped: bool = safety_status_bits.as_uint32()
            & (1 << SafetyStatusBits::IsProtectiveStopped as u32)
            != 0;
        Ok(is_protective_stopped)
    }

    pub fn is_connected(&self) -> bool {
        let rtde_lock = self.rtde.lock().unwrap();
        rtde_lock.is_connected()
    }

    pub fn disconnect(&mut self) -> Result<(), RTDEError> {
        self.stop_receive_thread.store(true, Ordering::Relaxed);
        if let Some(thread) = self.receive_thread.take() {
            thread.join().unwrap();
        }
        let mut rtde_lock = self.rtde.lock().unwrap();
        if rtde_lock.is_connected() {
            rtde_lock.disconnect(false)?;
        }
        Ok(())
    }
}

impl Drop for RTDEReceive {
    fn drop(&mut self) {
        self.disconnect().unwrap();
    }
}
