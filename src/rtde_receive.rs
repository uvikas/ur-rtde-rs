/* RTDE Receive Interface for reading data from UR robot */

use crate::robot_state::{RobotState, StateDataType};
use crate::rtde::RTDE;
use std::error::Error;

pub struct RTDEReceive {
    rtde: RTDE,
    robot_state: RobotState,
}

impl RTDEReceive {
    pub fn new(hostname: String) -> Self {
        Self { rtde: RTDE::new(hostname), robot_state: RobotState::new() }
    }

    pub fn get_actual_q(&self) -> Result<[f32; 6], Box<dyn Error>> {
        let actual_q: [f32; 6] = self.robot_state.get_state_data(StateDataType::ActualQ)?;
        Ok(actual_q)
    }

    pub fn get_actual_qd(&self) -> Result<[f32; 6], Box<dyn Error>> {
        let actual_qd: [f32; 6] = self.robot_state.get_state_data(StateDataType::ActualQd)?;
        Ok(actual_qd)
    }

    pub fn get_actual_tcp_pose(&self) -> Result<[f32; 6], Box<dyn Error>> {
        let actual_tcp_pose: [f32; 6] =
            self.robot_state.get_state_data(StateDataType::ActualTcpPose)?;
        Ok(actual_tcp_pose)
    }

    pub fn get_actual_tcp_speed(&self) -> Result<[f32; 6], Box<dyn Error>> {
        let actual_tcp_speed: [f32; 6] =
            self.robot_state.get_state_data(StateDataType::ActualTcpSpeed)?;
        Ok(actual_tcp_speed)
    }

    pub fn get_actual_tcp_force(&self) -> Result<[f32; 6], Box<dyn Error>> {
        let actual_tcp_force: [f32; 6] =
            self.robot_state.get_state_data(StateDataType::ActualTcpForce)?;
        Ok(actual_tcp_force)
    }

    pub fn is_protective_stopped(&self) -> Result<bool, Box<dyn Error>> {
        let is_protective_stopped: bool =
            self.robot_state.get_state_data(StateDataType::IsProtectiveStopped)?;
        Ok(is_protective_stopped)
    }

    pub fn get_robot_mode(&self) -> Result<u8, Box<dyn Error>> {
        let robot_mode: u8 = self.robot_state.get_state_data(StateDataType::RobotMode)?;
        Ok(robot_mode)
    }

    pub fn disconnect(&mut self) -> Result<(), Box<dyn Error>> {
        self.rtde.disconnect(false)
    }
}
