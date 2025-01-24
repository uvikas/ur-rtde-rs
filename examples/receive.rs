use ur_rtde::rtde_receive::RTDEReceive;

fn test_rtde_receive() {
    let rtde = RTDEReceive::new("10.42.0.100").unwrap();

    loop {
        let robot_joints = rtde.get_actual_q().unwrap();
        println!("Robot joints: {:?}", robot_joints);
        std::thread::sleep(std::time::Duration::from_millis(100));
    }
}

fn main() {
    test_rtde_receive();
}
