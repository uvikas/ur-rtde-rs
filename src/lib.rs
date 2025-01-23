mod common;
mod dashboard;
mod robot_state;
mod rtde;
mod rtde_control;
mod rtde_receive;

pub use log;

pub fn init_logging() {
    if let Err(_) = std::env::var("RUST_LOG") {
        std::env::set_var("RUST_LOG", "info");
    }

    env_logger::builder().try_init().ok();
}
