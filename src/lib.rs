mod common;
mod dashboard;
mod robot_state;
mod rtde;
pub mod rtde_control;
pub mod rtde_receive;
mod script_client;
mod ur_script;
mod utils;

pub use log;

pub fn init_logging() {
    if let Err(_) = std::env::var("RUST_LOG") {
        std::env::set_var("RUST_LOG", "info");
    }

    env_logger::builder().try_init().ok();
}
