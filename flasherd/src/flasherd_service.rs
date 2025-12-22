use crate::flasherd::flasherd_server::Flasherd;
use crate::flasherd::{RunCommandRequest, RunCommandResponse};
use anyhow::Result;
use futures::executor::block_on;
use log::{error, info};
use std::env::args;
use std::mem::replace;
use std::path::PathBuf;
use std::pin::Pin;
use std::process::Stdio;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::process::Command;
use tokio::sync::mpsc;
use tokio_stream::wrappers::ReceiverStream;
use tokio_stream::{Stream, StreamExt};
use tonic::{Request, Response, Status, Streaming};

/// Reports an error and closes the response stream.
async fn send_err_msg(tx: mpsc::Sender<Result<RunCommandResponse, Status>>, status: Status) {
    error!("Reporting error: {status}");
    _ = tx.send(Err(status)).await;
    drop(tx); // Close stream
}

pub struct FlasherdService {
    clover_root: PathBuf,
}

impl FlasherdService {
    pub(crate) fn new() -> Self {
        let args = args();
        info!("Received args: {args:?}");
        if args.len() != 2 {
            panic!("Invalid arguments: {args:?}");
        }
        let clover_root = PathBuf::from(args.skip(1).next().unwrap());
        info!("arty root: {clover_root:?}");
        Self { clover_root }
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

        let clover_root = self.clover_root.clone();
        tokio::spawn(async move {
            // Initial packet should specify only command
            let Some(packet) = req_stream.next().await else {
                return;
            };
            let request = match packet {
                Err(err) => {
                    send_err_msg(resp_tx, err).await;
                    return;
                }
                Ok(request) => request,
            };
            info!("Received command packet: {request:#?}");

            if let Some(ref stdin) = request.stdin {
                send_err_msg(
                    resp_tx,
                    Status::invalid_argument(format!(
                        "First packet must have empty stdin, got: `{}`",
                        String::from_utf8_lossy(stdin)
                    )),
                )
                .await;
                return;
            }

            // Convert first packet to actual command and args
            let command = if cfg!(target_os = "windows") {
                let Some(command) = request.command_windows else {
                    send_err_msg(resp_tx, Status::invalid_argument("Missing Windows command"))
                        .await;
                    return;
                };
                command
            } else if cfg!(target_os = "linux") {
                let Some(command) = request.command_linux else {
                    send_err_msg(resp_tx, Status::invalid_argument("Missing Linux command")).await;
                    return;
                };
                command
            } else if cfg!(target_os = "macos") {
                let Some(command) = request.command_macos else {
                    send_err_msg(resp_tx, Status::invalid_argument("Missing macOS command")).await;
                    return;
                };
                command
            } else {
                panic!("Unknown target os");
            };
            let Ok(args): Result<Vec<String>, _> = request
                .args
                .into_iter()
                .map(|arg| match (arg.regular, arg.path) {
                    (Some(regular_arg), None) => Ok(regular_arg),
                    (None, Some(path_arg)) => {
                        Ok(clover_root.join(path_arg).to_string_lossy().to_string())
                    }
                    _ => Err(()),
                })
                .collect()
            else {
                block_on(send_err_msg(
                    resp_tx,
                    Status::invalid_argument("Invalid arguments field"),
                ));
                return;
            };

            // Spawn command process
            info!("Spawning: {:?}", command);
            let child = Command::new(command)
                .args(args)
                .stdin(Stdio::piped())
                .stdout(Stdio::piped())
                .stderr(Stdio::piped())
                .spawn();
            let mut child = match child {
                Err(err) => {
                    send_err_msg(
                        resp_tx,
                        Status::internal(format!("Failed to spawn command: {err}")),
                    )
                    .await;
                    return;
                }
                Ok(child) => child,
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
                                    Status::invalid_argument(
                                        "Stdin request must have non-empty stdin",
                                    ),
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
                            if let Err(err) = resp_tx3
                                .send(Ok(RunCommandResponse {
                                    stdout: Some(replace(&mut message, Vec::new())),
                                    ..Default::default()
                                }))
                                .await
                            {
                                error!("Failed to send response: {err}");
                                return;
                            }
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
                            if let Err(err) = resp_tx3
                                .send(Ok(RunCommandResponse {
                                    stderr: Some(replace(&mut message, Vec::new())),
                                    ..Default::default()
                                }))
                                .await
                            {
                                error!("Failed to send response: {err}");
                                return;
                            }
                        }
                    }
                }
            });

            // Await process death
            let _ = tokio::spawn(async move {
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

                        if let Err(err) = resp_tx
                            .send(Ok(RunCommandResponse {
                                exit_code: Some(
                                    code.code().unwrap_or(255).try_into().unwrap_or(255),
                                ),
                                ..Default::default()
                            }))
                            .await
                        {
                            error!("Failed to send response: {err}");
                            return;
                        }
                    }
                }
            });
        });

        Ok(Response::new(
            Box::pin(ReceiverStream::new(resp_rx)) as Self::RunCommandStream
        ))
    }
}
