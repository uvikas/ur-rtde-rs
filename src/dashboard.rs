use crate::rtde::RTDEError;
use log::{debug, info};
use std::net::SocketAddr;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::TcpStream;

#[derive(Debug, PartialEq, Eq)]
enum ConnectionState {
    Disconnected,
    Connected,
}

pub struct DashboardClient {
    hostname: String,
    port: u16,
    conn_state: ConnectionState,
    reader: Option<BufReader<TcpStream>>,
}

impl DashboardClient {
    pub fn new(hostname: &str) -> Self {
        Self {
            hostname: hostname.to_string(),
            port: 29999,
            conn_state: ConnectionState::Disconnected,
            reader: None,
        }
    }

    pub async fn connect(&mut self) -> Result<(), RTDEError> {
        let addr = format!("{}:{}", self.hostname, self.port)
            .parse::<SocketAddr>()
            .map_err(|e| format!("Invalid address: {}", e))?;

        let stream = TcpStream::connect(&addr).await?;
        stream.set_nodelay(true)?;

        self.conn_state = ConnectionState::Connected;
        self.reader = Some(BufReader::new(stream));

        let intro_msg = self.receive().await?;
        debug!("Intro msg {:?}", intro_msg);
        Ok(())
    }

    pub async fn is_connected(&self) -> bool {
        self.conn_state == ConnectionState::Connected
    }

    pub async fn disconnect(&mut self) -> Result<(), RTDEError> {
        if let Some(reader) = self.reader.take() {
            // Get the inner stream from the reader
            let mut stream = reader.into_inner();
            stream.shutdown().await?;
        }
        self.conn_state = ConnectionState::Disconnected;
        Ok(())
    }

    pub async fn send(&mut self, str: &str) -> Result<(), RTDEError> {
        if let Some(reader) = &mut self.reader {
            // Get a mutable reference to the underlying stream
            let stream = reader.get_mut();
            stream.write_all(str.as_bytes()).await?;
            return Ok(());
        }
        Err(RTDEError::ConnectionError("Dashboard stream is not connected".into()))
    }

    pub async fn receive(&mut self) -> Result<String, RTDEError> {
        if let Some(reader) = &mut self.reader {
            let mut buffer = Vec::new();
            reader.read_until(b'\n', &mut buffer).await?;
            return Ok(String::from_utf8(buffer).unwrap());
        }
        Err(RTDEError::ConnectionError("Dashboard stream is not connected".into()))
    }

    // Dashboard commands

    pub async fn stop(&mut self) -> Result<(), RTDEError> {
        self.send("stop\n").await?;
        let result = self.receive().await?;
        if result != "Stopped\n" {
            return Err(RTDEError::DashboardError(format!("Failed to stop: {}", result)));
        }
        Ok(())
    }

    pub async fn is_in_remote_control(&mut self) -> Result<bool, RTDEError> {
        self.send("is in remote control\n").await?;
        let result = self.receive().await?;
        debug!("is_in_remote_control: {}", result);
        Ok(result == "true\n")
    }
}
