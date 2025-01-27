use tokio::time::Duration;
use ur_rtde::init_logging;
use ur_rtde::rtde_control::RTDEControl;
use ur_rtde::rtde_receive::RTDEReceive;

use log::info;
use nalgebra as na;

async fn test_rtde_control() {
    let receive = RTDEReceive::new("10.42.1.100").await.unwrap();
    let control = RTDEControl::new("10.42.1.100").await.unwrap();

    info!("Connected to robot");

    loop {
        let robot_joints = &receive.get_actual_q().await.unwrap();
        println!("Robot joints: {:?}", robot_joints);

        // Convert array to nalgebra Vector6 and add 0.1 to each element
        let joint_pos = na::Vector6::from_row_slice(robot_joints) + na::Vector6::repeat(0.1);
        println!("Modified joints: {:?}", joint_pos);

        tokio::time::sleep(Duration::from_millis(2)).await;
    }
}

#[tokio::main(flavor = "current_thread")]
async fn main() {
    init_logging();
    test_rtde_control().await;
}
