use tokio::time::Duration;
use ur_rtde::init_logging;
use ur_rtde::rtde_control::RTDEControl;
use ur_rtde::rtde_receive::RTDEReceive;

use log::info;

async fn test_rtde_control() {
    // let receive = RTDEReceive::new("10.42.1.100").await.unwrap();
    let control = RTDEControl::new("10.42.1.100").await.unwrap();

    // loop {
    //     let robot_joints = receive.get_actual_q().await.unwrap();
    //     println!("Robot joints: {:?}", robot_joints);
    //     tokio::time::sleep(Duration::from_millis(100)).await;
    // }

    info!("Connected to robot");
}

#[tokio::main(flavor = "current_thread")]
async fn main() {
    init_logging();
    test_rtde_control().await;
}
