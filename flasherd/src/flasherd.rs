mod flasherd_service;
pub mod map_tonic_err;
mod pid_path;

pub mod flasherd {
    tonic::include_proto!("flasherd");
}

use crate::flasherd::flasherd_server::FlasherdServer;
use anyhow::{Context, Result};
use dirs::data_dir;
use flasherd_service::FlasherdService;
use log::{LevelFilter, info, warn};
use pid_path::pid_path;
use rand::distr::{Alphanumeric, SampleString};
use rand::rng;
use rand_distr::num_traits::ToPrimitive;
use simplelog::{ColorChoice, CombinedLogger, Config, TermLogger, TerminalMode, WriteLogger};
use std::process;
use std::process::{Stdio, exit};
use sysinfo::{Pid, System};
use tokio::fs::{create_dir_all, write};
use tokio::process::Command;
use tonic::transport::Server;

const FLASHERD_PORT: u16 = 6767;

#[tokio::main]
async fn main() -> Result<()> {
    // Determine if we should attempt to detach.
    daemonize().await;

    // Prepare logging.
    let log_dir = data_dir()
        .context("Failed to find data dir")?
        .join("flasherd");
    create_dir_all(&log_dir).await?;

    let log_file_name = format!(
        "{}-{}.log",
        chrono::offset::Local::now().format("%Y-%m-%d_%H-%M-%S"),
        Alphanumeric.sample_string(&mut rng(), 6)
    );
    let log_file_path = log_dir.join(log_file_name);
    let log_file = std::fs::File::create(&log_file_path)?;

    CombinedLogger::init(vec![
        TermLogger::new(
            LevelFilter::Info,
            Config::default(),
            TerminalMode::Mixed,
            ColorChoice::Auto,
        ),
        WriteLogger::new(LevelFilter::Debug, Config::default(), log_file),
    ])
    .expect("Failed to initialize logging.");
    info!("Logging to {log_file_path:?}");

    // Spawn service.
    let service = FlasherdService::new()
        .await
        .expect("Failed to create service");
    let res = Server::builder()
        .add_service(FlasherdServer::new(service))
        .serve(format!("127.0.0.1:{FLASHERD_PORT}").parse()?)
        .await;
    warn!("Terminated with: {res:?}");

    Ok(())
}

/// If --daemonize is passed as an arg, respawn flasherd as a detatched process.
async fn daemonize() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 2 || args[1] != "--daemonize" {
        return;
    }
    let sys = System::new_all();

    // Kill old process, if it exists

    let self_bin_path = sys
        .process(Pid::from(process::id().to_usize().unwrap()))
        .unwrap()
        .exe()
        .unwrap();
    let child = Command::new(self_bin_path)
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .unwrap();

    let pid_path = pid_path();
    write(pid_path, child.id().unwrap().to_string())
        .await
        .unwrap();
    println!("Spawned flasherd with pid: {}", child.id().unwrap());
    exit(0);
}
