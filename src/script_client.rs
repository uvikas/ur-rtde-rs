use crate::rtde::RTDEError;
use crate::ur_script::UR_SCRIPT;
use log::{debug, error, info};
use std::net::SocketAddr;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;

#[derive(Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum ConnectionState {
    Disconnected = 0,
    Connected = 1,
}

#[derive(Debug, PartialEq, Eq)]
pub struct ScriptInjectItem {
    search_string: String,
    inject_string: String,
}

pub struct ScriptClient {
    hostname: String,
    port: u16,
    conn_state: ConnectionState,

    stream: Option<TcpStream>,
    script_injections: Vec<ScriptInjectItem>,
    ur_script: Option<String>,
}

impl ScriptClient {
    pub fn new(hostname: &str) -> Self {
        Self {
            hostname: hostname.to_string(),
            port: 30003,
            conn_state: ConnectionState::Disconnected,
            stream: None,
            script_injections: Vec::new(),
            ur_script: None,
        }
    }

    pub async fn connect(&mut self) -> Result<(), RTDEError> {
        let addr = format!("{}:{}", self.hostname, self.port)
            .parse::<SocketAddr>()
            .map_err(|e| format!("Invalid address: {}", e))?;

        let stream = TcpStream::connect(&addr).await?;
        stream.set_nodelay(true)?;

        self.stream = Some(stream);
        self.conn_state = ConnectionState::Connected;

        debug!("Connected to robot!");
        Ok(())
    }

    pub async fn disconnect(&mut self) -> Result<(), RTDEError> {
        if let Some(mut stream) = self.stream.take() {
            stream.shutdown().await?;
        }
        self.conn_state = ConnectionState::Disconnected;
        debug!("Disconnected from robot");
        Ok(())
    }

    pub fn is_connected(&self) -> bool {
        self.conn_state == ConnectionState::Connected
    }

    pub async fn send_script(&mut self) -> Result<(), RTDEError> {
        let mut ur_script = String::new();

        if let Some(script) = self.ur_script.take() {
            todo!()
        }

        if ur_script.is_empty() {
            ur_script = "def rtde_control():\n".to_string();
            ur_script += UR_SCRIPT;
            ur_script += "end\n";
        }

        // self.scan_and_inject_additional_script_code(&mut ur_script);

        // info!("Script:==========\n{}\n==========", ur_script);

        if self.is_connected() && !ur_script.is_empty() {
            self.stream.as_mut().unwrap().write_all(ur_script.as_bytes()).await?;
            return Ok(());
        }

        Err(RTDEError::ScriptClientError("Failed to send script".to_string()))
    }

    pub async fn send_script_command(&mut self, cmd_str: &str) -> Result<(), RTDEError> {
        todo!()
    }

    pub async fn stop_script(&mut self) -> Result<(), RTDEError> {
        todo!()
    }

    pub fn set_script_injection(&mut self, search_string: &str, inject_string: &str) {
        if let Some(item) =
            self.script_injections.iter_mut().find(|item| item.search_string == search_string)
        {
            item.inject_string = inject_string.to_string();
        } else {
            self.script_injections.push(ScriptInjectItem {
                search_string: search_string.to_string(),
                inject_string: inject_string.to_string(),
            });
        }
    }

    fn scan_and_inject_additional_script_code(&mut self, ur_script: &mut String) {
        let mut n: usize;

        for item in self.script_injections.iter() {
            n = ur_script.find(&item.search_string).unwrap_or(usize::MAX);
            if n == usize::MAX {
                error!("script_injection [{}] not found in script", item.search_string);
                continue;
            }

            ur_script.insert_str(n + item.search_string.len(), item.inject_string.as_str());
        }
    }

    pub fn get_script(&mut self) -> Result<String, RTDEError> {
        Ok(String::new())
    }
}
