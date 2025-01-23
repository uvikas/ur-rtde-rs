/* Robot State */

use std::collections::HashMap;
use std::error::Error;

#[derive(Hash, Eq, PartialEq, Debug)]
pub enum StateData {
    ActualQ,
    ActualQd,
    ActualTcpPose,
    ActualTcpSpeed,
    ActualTcpForce,
    IsProtectiveStopped,
    RobotMode,
}

impl StateData {
    pub fn as_str(&self) -> &str {
        match self {
            StateData::ActualQ => "ActualQ",
            StateData::ActualQd => "ActualQd",
            StateData::ActualTcpPose => "ActualTcpPose",
            StateData::ActualTcpSpeed => "ActualTcpSpeed",
            StateData::ActualTcpForce => "ActualTcpForce",
            StateData::IsProtectiveStopped => "IsProtectiveStopped",
            StateData::RobotMode => "RobotMode",
        }
    }
}

#[derive(Debug)]
enum StateDataTypes {
    FloatArray([f32; 6]),
    Boolean(bool),
    Int(i32),
}

pub struct RobotState {
    state_data: HashMap<StateData, StateDataTypes>,
}

impl RobotState {
    pub fn new(variables: Vec<StateData>) -> Self {
        let mut robot_state = Self { state_data: HashMap::new() };
        robot_state.init_robot_state(variables)?;
        robot_state
    }

    pub fn get_state_data<T>(&self, state_data_type: StateData) -> Result<T, Box<dyn Error>> {
        todo!()
    }

    pub fn set_state_data<T>(
        &self,
        state_data_type: StateData,
        data: T,
    ) -> Result<(), Box<dyn Error>> {
        todo!()
    }

    pub fn init_robot_state(&self, variables: Vec<StateData>) -> Result<(), Box<dyn Error>> {
        todo!()
    }
}
