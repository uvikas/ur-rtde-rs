// Shared Robot State

use log::debug;
use phf::phf_map;
use std::collections::HashMap;
use std::error::Error;

use crate::rtde::RTDEError;

// State data types to store in RobotState.
#[derive(Debug, Clone)]
pub enum StateDataTypes {
    VectorDouble(Vec<f64>),
    VectorInt(Vec<i32>),
    Boolean(bool),
    Uint32(u32),
    Double(f64),
    Uint64(u64),
    Int32(i32),
}

impl StateDataTypes {
    pub fn as_vec_double(&self) -> Vec<f64> {
        match self {
            StateDataTypes::VectorDouble(x) => x.clone(),
            _ => Vec::new(),
        }
    }

    pub fn as_vec_int(&self) -> Vec<i32> {
        match self {
            StateDataTypes::VectorInt(x) => x.clone(),
            _ => Vec::new(),
        }
    }

    pub fn as_bool(&self) -> bool {
        match self {
            StateDataTypes::Boolean(x) => x.clone(),
            _ => false,
        }
    }

    pub fn as_uint32(&self) -> u32 {
        match self {
            StateDataTypes::Uint32(x) => x.clone(),
            _ => 0,
        }
    }

    pub fn as_double(&self) -> f64 {
        match self {
            StateDataTypes::Double(x) => x.clone(),
            _ => 0.0,
        }
    }

    pub fn as_uint64(&self) -> u64 {
        match self {
            StateDataTypes::Uint64(x) => x.clone(),
            _ => 0,
        }
    }

    pub fn as_int32(&self) -> i32 {
        match self {
            StateDataTypes::Int32(x) => x.clone(),
            _ => 0,
        }
    }
}

const STATE_DATA_TYPES: phf::Map<&str, StateDataTypes> = phf_map! {
    "timestamp" => StateDataTypes::Double(0.0),
    "target_q" => StateDataTypes::VectorDouble(Vec::new()),
    "target_qd" => StateDataTypes::VectorDouble(Vec::new()),
    "target_qdd" => StateDataTypes::VectorDouble(Vec::new()),
    "target_current" => StateDataTypes::VectorDouble(Vec::new()),
    "target_moment" => StateDataTypes::VectorDouble(Vec::new()),
    "actual_q" => StateDataTypes::VectorDouble(Vec::new()),
    "actual_qd" => StateDataTypes::VectorDouble(Vec::new()),
    "actual_qdd" => StateDataTypes::VectorDouble(Vec::new()),
    "actual_current" => StateDataTypes::VectorDouble(Vec::new()),
    "joint_control_output" => StateDataTypes::VectorDouble(Vec::new()),
    "actual_TCP_pose" => StateDataTypes::VectorDouble(Vec::new()),
    "actual_TCP_speed" => StateDataTypes::VectorDouble(Vec::new()),
    "actual_TCP_force" => StateDataTypes::VectorDouble(Vec::new()),
    "target_TCP_pose" => StateDataTypes::VectorDouble(Vec::new()),
    "target_TCP_speed" => StateDataTypes::VectorDouble(Vec::new()),
    "actual_digital_input_bits" => StateDataTypes::Uint64(0),
    "joint_temperatures" => StateDataTypes::VectorDouble(Vec::new()),
    "actual_execution_time" => StateDataTypes::Double(0.0),
    "robot_mode" => StateDataTypes::Uint32(0),
    "joint_mode" => StateDataTypes::VectorInt(Vec::new()),
    "safety_mode" => StateDataTypes::Uint32(0),
    "actual_tool_accelerometer" => StateDataTypes::VectorDouble(Vec::new()),
    "speed_scaling" => StateDataTypes::Double(0.0),
    "target_speed_fraction" => StateDataTypes::Double(0.0),
    "actual_momentum" => StateDataTypes::Double(0.0),
    "actual_main_voltage" => StateDataTypes::Double(0.0),
    "actual_robot_voltage" => StateDataTypes::Double(0.0),
    "actual_robot_current" => StateDataTypes::Double(0.0),
    "actual_joint_voltage" => StateDataTypes::VectorDouble(Vec::new()),
    "actual_digital_output_bits" => StateDataTypes::Uint64(0),
    "runtime_state" => StateDataTypes::Uint32(0),
    "robot_status_bits" => StateDataTypes::Uint32(0),
    "safety_status_bits" => StateDataTypes::Uint32(0),
    "standard_analog_input0" => StateDataTypes::Double(0.0),
    "standard_analog_input1" => StateDataTypes::Double(0.0),
    "standard_analog_output0" => StateDataTypes::Double(0.0),
    "standard_analog_output1" => StateDataTypes::Double(0.0),
    "ft_raw_wrench" => StateDataTypes::VectorDouble(Vec::new()),
    "payload" => StateDataTypes::Double(0.0),
    "payload_cog" => StateDataTypes::VectorDouble(Vec::new()),
    "payload_inertia" => StateDataTypes::VectorDouble(Vec::new()),
    "output_bit_registers0_to_31" => StateDataTypes::Uint32(0),
    "output_bit_registers32_to_63" => StateDataTypes::Uint32(0),
    "output_int_register_0" => StateDataTypes::Int32(0),
    "output_int_register_1" => StateDataTypes::Int32(0),
    "output_int_register_2" => StateDataTypes::Int32(0),
    "output_int_register_3" => StateDataTypes::Int32(0),
    "output_int_register_4" => StateDataTypes::Int32(0),
    "output_int_register_5" => StateDataTypes::Int32(0),
    "output_int_register_6" => StateDataTypes::Int32(0),
    "output_int_register_7" => StateDataTypes::Int32(0),
    "output_int_register_8" => StateDataTypes::Int32(0),
    "output_int_register_9" => StateDataTypes::Int32(0),
    "output_int_register_10" => StateDataTypes::Int32(0),
    "output_int_register_11" => StateDataTypes::Int32(0),
    "output_int_register_12" => StateDataTypes::Int32(0),
    "output_int_register_13" => StateDataTypes::Int32(0),
    "output_int_register_14" => StateDataTypes::Int32(0),
    "output_int_register_15" => StateDataTypes::Int32(0),
    "output_int_register_16" => StateDataTypes::Int32(0),
    "output_int_register_17" => StateDataTypes::Int32(0),
    "output_int_register_18" => StateDataTypes::Int32(0),
    "output_int_register_19" => StateDataTypes::Int32(0),
    "output_int_register_20" => StateDataTypes::Int32(0),
    "output_int_register_21" => StateDataTypes::Int32(0),
    "output_int_register_22" => StateDataTypes::Int32(0),
    "output_int_register_23" => StateDataTypes::Int32(0),
    "output_int_register_24" => StateDataTypes::Int32(0),
    "output_int_register_25" => StateDataTypes::Int32(0),
    "output_int_register_26" => StateDataTypes::Int32(0),
    "output_int_register_27" => StateDataTypes::Int32(0),
    "output_int_register_28" => StateDataTypes::Int32(0),
    "output_int_register_29" => StateDataTypes::Int32(0),
    "output_int_register_30" => StateDataTypes::Int32(0),
    "output_int_register_31" => StateDataTypes::Int32(0),
    "output_int_register_32" => StateDataTypes::Int32(0),
    "output_int_register_33" => StateDataTypes::Int32(0),
    "output_int_register_34" => StateDataTypes::Int32(0),
    "output_int_register_35" => StateDataTypes::Int32(0),
    "output_int_register_36" => StateDataTypes::Int32(0),
    "output_int_register_37" => StateDataTypes::Int32(0),
    "output_int_register_38" => StateDataTypes::Int32(0),
    "output_int_register_39" => StateDataTypes::Int32(0),
    "output_int_register_40" => StateDataTypes::Int32(0),
    "output_int_register_41" => StateDataTypes::Int32(0),
    "output_int_register_42" => StateDataTypes::Int32(0),
    "output_int_register_43" => StateDataTypes::Int32(0),
    "output_int_register_44" => StateDataTypes::Int32(0),
    "output_int_register_45" => StateDataTypes::Int32(0),
    "output_int_register_46" => StateDataTypes::Int32(0),
    "output_int_register_47" => StateDataTypes::Int32(0),
    "output_double_register_0" => StateDataTypes::Double(0.0),
    "output_double_register_1" => StateDataTypes::Double(0.0),
    "output_double_register_2" => StateDataTypes::Double(0.0),
    "output_double_register_3" => StateDataTypes::Double(0.0),
    "output_double_register_4" => StateDataTypes::Double(0.0),
    "output_double_register_5" => StateDataTypes::Double(0.0),
    "output_double_register_6" => StateDataTypes::Double(0.0),
    "output_double_register_7" => StateDataTypes::Double(0.0),
    "output_double_register_8" => StateDataTypes::Double(0.0),
    "output_double_register_9" => StateDataTypes::Double(0.0),
    "output_double_register_10" => StateDataTypes::Double(0.0),
    "output_double_register_11" => StateDataTypes::Double(0.0),
    "output_double_register_12" => StateDataTypes::Double(0.0),
    "output_double_register_13" => StateDataTypes::Double(0.0),
    "output_double_register_14" => StateDataTypes::Double(0.0),
    "output_double_register_15" => StateDataTypes::Double(0.0),
    "output_double_register_16" => StateDataTypes::Double(0.0),
    "output_double_register_17" => StateDataTypes::Double(0.0),
    "output_double_register_18" => StateDataTypes::Double(0.0),
    "output_double_register_19" => StateDataTypes::Double(0.0),
    "output_double_register_20" => StateDataTypes::Double(0.0),
    "output_double_register_21" => StateDataTypes::Double(0.0),
    "output_double_register_22" => StateDataTypes::Double(0.0),
    "output_double_register_23" => StateDataTypes::Double(0.0),
    "output_double_register_24" => StateDataTypes::Double(0.0),
    "output_double_register_25" => StateDataTypes::Double(0.0),
    "output_double_register_26" => StateDataTypes::Double(0.0),
    "output_double_register_27" => StateDataTypes::Double(0.0),
    "output_double_register_28" => StateDataTypes::Double(0.0),
    "output_double_register_29" => StateDataTypes::Double(0.0),
    "output_double_register_30" => StateDataTypes::Double(0.0),
    "output_double_register_31" => StateDataTypes::Double(0.0),
    "output_double_register_32" => StateDataTypes::Double(0.0),
    "output_double_register_33" => StateDataTypes::Double(0.0),
    "output_double_register_34" => StateDataTypes::Double(0.0),
    "output_double_register_35" => StateDataTypes::Double(0.0),
    "output_double_register_36" => StateDataTypes::Double(0.0),
    "output_double_register_37" => StateDataTypes::Double(0.0),
    "output_double_register_38" => StateDataTypes::Double(0.0),
    "output_double_register_39" => StateDataTypes::Double(0.0),
    "output_double_register_40" => StateDataTypes::Double(0.0),
    "output_double_register_41" => StateDataTypes::Double(0.0),
    "output_double_register_42" => StateDataTypes::Double(0.0),
    "output_double_register_43" => StateDataTypes::Double(0.0),
    "output_double_register_44" => StateDataTypes::Double(0.0),
    "output_double_register_45" => StateDataTypes::Double(0.0),
    "output_double_register_46" => StateDataTypes::Double(0.0),
    "output_double_register_47" => StateDataTypes::Double(0.0),
};

pub struct RobotState {
    state_data: HashMap<String, StateDataTypes>,
    first_state_received: bool,
}

impl RobotState {
    // Create new RobotState.
    pub fn new(fields: &Vec<&str>) -> Self {
        let mut robot_state = Self { state_data: HashMap::new(), first_state_received: false };
        robot_state.init_robot_state(fields);
        robot_state
    }

    // Get first state received.
    pub fn get_first_state_received(&self) -> bool {
        self.first_state_received
    }

    // Set first state received.
    pub fn set_first_state_received(&mut self, val: bool) {
        self.first_state_received = val;
    }

    // Get state data.
    pub fn get_state_data(&self, state_data_type: &str) -> Result<StateDataTypes, RTDEError> {
        let val = self.state_data.get(state_data_type);
        if val.is_none() {
            return Err(RTDEError::NoDataAvailable(format!(
                "State data not found: {}",
                state_data_type
            )));
        }
        Ok(val.unwrap().clone()) // TODO: is clone necessary?
    }

    // Set state data.
    pub fn set_state_data(
        &mut self,
        state_data_type: String,
        data: StateDataTypes,
    ) -> Result<(), Box<dyn Error>> {
        if self.state_data.contains_key(&state_data_type) {
            self.state_data.insert(state_data_type, data);
        } else {
            return Err(Box::new(std::io::Error::new(
                std::io::ErrorKind::NotFound,
                "State data not found",
            )));
        }
        Ok(())
    }

    // Initialize robot state.
    pub fn init_robot_state(&mut self, fields: &Vec<&str>) -> () {
        for field in fields {
            self.state_data.insert(field.to_string(), STATE_DATA_TYPES.get(field).unwrap().clone());
        }
    }
}
