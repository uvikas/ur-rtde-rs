use tokio::time::Duration;
use ur_rtde::init_logging;
use ur_rtde::rtde_receive::RTDEReceive;

async fn test_rtde_receive() {
    let rtde = RTDEReceive::new("10.42.1.100").await.unwrap();

    loop {
        let robot_joints = rtde.get_actual_q().await.unwrap();
        println!("Robot joints: {:?}", robot_joints);
        tokio::time::sleep(Duration::from_millis(100)).await;
    }
}

#[tokio::main(flavor = "current_thread")]
async fn main() {
    init_logging();
    test_rtde_receive().await;
}
