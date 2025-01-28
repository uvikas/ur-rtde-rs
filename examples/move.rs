use tokio::time::Duration;
use ur_rtde::init_logging;
use ur_rtde::rtde_control::RTDEControl;
use ur_rtde::rtde_receive::RTDEReceive;

use log::info;

use nalgebra as na;

async fn test_rtde_control() {
    let receive = RTDEReceive::new("10.42.1.100").await.unwrap();
    let mut control = RTDEControl::new("10.42.1.100").await.unwrap();

    info!("Connected to robot");

    loop {
        let robot_joints = receive.get_actual_q().await.unwrap();
        println!("Robot joints: {:?}", robot_joints);

        let mut joints = na::Vector6::from_row_slice(&robot_joints);
        joints = joints + na::Vector6::repeat(0.01);

        control.servo_j(&joints.as_slice(), 0.5, 0.5, 1.0 / 500.0, 0.1, 200.0).await.unwrap();
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}

#[tokio::main(flavor = "current_thread")]
async fn main() {
    init_logging();
    test_rtde_control().await;
}
