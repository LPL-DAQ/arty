use crate::flasherd::flasherd_server::Flasherd;
use crate::flasherd::{RunCommandRequest, RunCommandResponse};
use crate::map_tonic_err::MapTonicErr;
use anyhow::{Context, Result, anyhow, bail};
use log::{error, info};
use std::cmp::max;
use std::mem::replace;
use std::path::PathBuf;
use std::pin::Pin;
use std::process::Stdio;
use std::sync::atomic::AtomicU64;
use std::sync::atomic::Ordering::SeqCst;
use tokio::fs::{OpenOptions, create_dir_all, read_dir};
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::process::Command;
use tokio::sync::mpsc;
use tokio_stream::wrappers::ReceiverStream;
use tokio_stream::{Stream, StreamExt};
use tonic::{Code, Request, Response, Status, Streaming};

const INITIAL_REQ_ID: u64 = 67;

/// Reports an error and closes the response stream.
async fn send_err_msg(tx: mpsc::Sender<Result<RunCommandResponse, Status>>, status: Status) {
    error!("Reporting error: {status}");
    _ = tx.send(Err(status)).await;
    drop(tx); // Close stream
}

pub struct FlasherdService {
    next_req_id: AtomicU64,
    binaries_dir: PathBuf,
}

impl FlasherdService {
    pub(crate) async fn new() -> Result<Self> {
        let binaries_dir = dirs::data_local_dir()
            .context("Failed to get data local dir")?
            .join("flasherd");
        create_dir_all(&binaries_dir)
            .await
            .context("Failed to ensure existence of registry dir")?;

        // Parse existing binaries for max request ID
        let mut max_req_id_seen = 0;
        let mut entries = read_dir(&binaries_dir)
            .await
            .context("Failed to read from registry dir")?;
        while let Some(entry) = entries
            .next_entry()
            .await
            .context("Failed to get next entry")?
        {
            let id: u64 = entry
                .file_name()
                .into_string()
                .map_err(|s| anyhow!("Failed to convert filename to string: {s:?}"))?
                .split_once('_')
                .context("Missing ID delimiter")
                .and_then(|(id, _)| id.parse().context("Failed to parse ID"))?;
            max_req_id_seen = max(max_req_id_seen, id);
        }

        Ok(Self {
            next_req_id: AtomicU64::new(max(INITIAL_REQ_ID, max_req_id_seen)),
            binaries_dir,
        })
    }
}

#[tonic::async_trait]
impl Flasherd for FlasherdService {
    type RunCommandStream = Pin<Box<dyn Stream<Item = Result<RunCommandResponse, Status>> + Send>>;

    async fn run_command(
        &self,
        request: Request<Streaming<RunCommandRequest>>,
    ) -> Result<Response<Self::RunCommandStream>, Status> {
        let mut req_stream = request.into_inner();
        let (resp_tx, resp_rx) = mpsc::channel(1024);
        let req_id = self.next_req_id.fetch_add(1, SeqCst);

        let mut child = loop {
            let req = req_stream
                .next()
                .await
                .context("Missing initial packets")
                .map_tonic_err(Code::InvalidArgument)?
                .context("Initial packet is err")
                .map_tonic_err(Code::InvalidArgument)?;

            info!("Received packet: {req:#?}");

            // Build binary file
            if let Some(binary_name) = req.binary_name {
                let binary_chunk = req
                    .binary_chunk
                    .context("Missing binary chunk")
                    .map_tonic_err(Code::InvalidArgument)?;

                let mut file = OpenOptions::new()
                    .write(true)
                    .append(true)
                    .create(true)
                    .open(format!("{req_id}_{binary_name}"))
                    .await
                    .context("Failed to open file")
                    .map_tonic_err(Code::Internal)?;
                file.write_all(&binary_chunk)
                    .await
                    .context("Failed to append chunk")
                    .map_tonic_err(Code::Internal)?;
            } else {
                let command = if cfg!(target_os = "windows") {
                    req.command_windows
                        .context("Missing windows command")
                        .map_tonic_err(Code::InvalidArgument)?
                } else if cfg!(target_os = "linux") {
                    req.command_linux
                        .context("Missing linux command")
                        .map_tonic_err(Code::InvalidArgument)?
                } else if cfg!(target_os = "macos") {
                    req.command_macos
                        .context("Missing macos command")
                        .map_tonic_err(Code::InvalidArgument)?
                } else {
                    return Err(Status::internal("Unknown OS"));
                };

                let args: Vec<_> = req
                    .args
                    .into_iter()
                    .map(|arg| match (arg.regular, arg.binary) {
                        (Some(regular), None) => Ok(regular),
                        (None, Some(binary)) => Ok(self
                            .binaries_dir
                            .join(format!("{req_id}_{binary}"))
                            .to_string_lossy()
                            .to_string()),
                        _ => bail!("Either regular or binary must be specified, but never both"),
                    })
                    .collect::<Result<_>>()
                    .map_tonic_err(Code::InvalidArgument)?;

                info!("Spawning `{command}` with args {args:?}");
                break Command::new(command)
                    .args(args)
                    .stdin(Stdio::piped())
                    .stdout(Stdio::piped())
                    .stderr(Stdio::piped())
                    .kill_on_drop(true)
                    .spawn()
                    .context("Failed to spawn command")
                    .map_tonic_err(Code::Internal)?;
            }
        };

        // Child is spawned with all stdio piped, this should never panic.
        let mut stdin = child.stdin.take().unwrap();
        let stdout = child.stdout.take().unwrap();
        let stderr = child.stderr.take().unwrap();

        // Stream stdin from request to process.
        let resp_tx2 = resp_tx.clone();
        let handle_stdin = tokio::spawn(async move {
            while let Some(packet) = req_stream.next().await {
                match packet {
                    Err(err) => {
                        send_err_msg(resp_tx2, err).await;
                        return;
                    }
                    Ok(request) => {
                        info!("Received stdin packet: {request:#?}");

                        if request.command_windows.is_some()
                            || request.command_linux.is_some()
                            || request.command_macos.is_some()
                            || !request.args.is_empty()
                        {
                            send_err_msg(
                                resp_tx2,
                                Status::invalid_argument(format!(
                                    "Stdin request cannot have a command, got: {:?}",
                                    request
                                )),
                            )
                            .await;
                            return;
                        }
                        let Some(message) = request.stdin else {
                            send_err_msg(
                                resp_tx2,
                                Status::invalid_argument("Stdin request must have non-empty stdin"),
                            )
                            .await;
                            return;
                        };

                        // If process is terminating
                        if let Err(_) = stdin.write_all(&message).await {
                            return;
                        }
                    }
                }
            }
        });

        // Stream stdout
        let resp_tx3 = resp_tx.clone();
        let handle_stdout = tokio::spawn(async move {
            let mut reader = BufReader::new(stdout);
            let mut message = Vec::new();
            loop {
                match reader.read_until(b'\n', &mut message).await {
                    Err(err) => {
                        send_err_msg(
                            resp_tx3,
                            Status::internal(format!("Failed to stream stdout: {err}")),
                        )
                        .await;
                        return;
                    }
                    Ok(bytes_read) => {
                        // Stream reached EOF
                        if bytes_read == 0 {
                            return;
                        }
                        _ = resp_tx3
                            .send(Ok(RunCommandResponse {
                                stdout: Some(replace(&mut message, Vec::new())),
                                ..Default::default()
                            }))
                            .await
                    }
                }
            }
        });

        // Stream stderr
        let resp_tx3 = resp_tx.clone();
        let handle_stderr = tokio::spawn(async move {
            let mut reader = BufReader::new(stderr);
            let mut message = Vec::new();
            loop {
                match reader.read_until(b'\n', &mut message).await {
                    Err(err) => {
                        send_err_msg(
                            resp_tx3,
                            Status::internal(format!("Failed to stream stdout: {err}")),
                        )
                        .await;
                        return;
                    }
                    Ok(bytes_read) => {
                        // Stream reached EOF
                        if bytes_read == 0 {
                            return;
                        }
                        _ = resp_tx3
                            .send(Ok(RunCommandResponse {
                                stderr: Some(replace(&mut message, Vec::new())),
                                ..Default::default()
                            }))
                            .await;
                    }
                }
            }
        });

        // Await process death
        _ = tokio::spawn(async move {
            let exit_code = child.wait().await;
            info!("Got status code: {exit_code:?}");
            match exit_code {
                Err(err) => {
                    send_err_msg(
                        resp_tx,
                        Status::internal(format!("Failed to get exit code: {err}")),
                    )
                    .await;
                    return;
                }
                Ok(code) => {
                    _ = handle_stdin.abort();
                    _ = handle_stdout.abort();
                    _ = handle_stderr.abort();

                    _ = resp_tx
                        .send(Ok(RunCommandResponse {
                            exit_code: Some(code.code().unwrap_or(255).try_into().unwrap_or(255)),
                            ..Default::default()
                        }))
                        .await;
                }
            }
        });

        Ok(Response::new(
            Box::pin(ReceiverStream::new(resp_rx)) as Self::RunCommandStream
        ))
    }
}
