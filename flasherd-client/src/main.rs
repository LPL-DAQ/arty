pub mod flasherd {
    tonic::include_proto!("flasherd");
}

use crate::flasherd::flasherd_client::FlasherdClient;
use crate::flasherd::{Arg, RunCommandRequest};
use futures::executor::block_on;
use std::env;
use std::io::BufRead;
use std::mem::replace;
use std::process::exit;
use tokio::io;
use tokio::io::AsyncWriteExt;
use tokio::sync::mpsc;
use tokio::task::spawn_blocking;
use tokio_stream::StreamExt;
use tokio_stream::wrappers::ReceiverStream;

const FLASHERD_PORT: u16 = 6767;

struct StdinGuard {}

impl Drop for StdinGuard {
    fn drop(&mut self) {
        exit(101);
    }
}

#[tokio::main]
async fn main() {
    // Skips tokio runtime graceful shutdown because stdin reader will hang indefinitely.
    let _exit_guard = StdinGuard {};

    println!(
        "[flasherd-client] Received args: {:?}",
        env::args().skip(1).collect::<Vec<String>>()
    );

    println!("[flasherd-client] Connecting to host flasherd at port {FLASHERD_PORT}");
    let mut client = FlasherdClient::connect(format!("http://localhost:{FLASHERD_PORT}"))
        .await
        .expect("Failed to connect to client");
    let (tx, rx) = mpsc::channel(128);

    // First packet starts command process
    let mut run_request = RunCommandRequest {
        ..Default::default()
    };
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        let val = args.next();
        assert!(val.is_some());
        match arg.as_str() {
            "--command-windows" => run_request.command_windows = val,
            "--command-macos" => run_request.command_macos = val,
            "--command-linux" => run_request.command_linux = val,
            "--arg-path" => {
                let val = val.unwrap();
                let relative_path = if let Some(path) = val.strip_prefix("~/arty/") {
                    path
                } else if let Some(path) = val.strip_prefix("/home/lpl/arty/") {
                    path
                } else if !val.starts_with("/") {
                    val.as_str()
                } else {
                    panic!("Path argument is not relative, or is not rooted at arty: {val}");
                };
                run_request.args.push(Arg {
                    path: Some(relative_path.to_string()),
                    ..Default::default()
                });
            }
            "--arg" => run_request.args.push(Arg {
                regular: val,
                ..Default::default()
            }),
            _ => panic!("Unexpected argument: {arg}"),
        }
    }
    tx.send(run_request)
        .await
        .expect("Failed to send first request packet");

    // Stream stdin
    spawn_blocking(move || {
        let stdin = std::io::stdin();
        let mut buf = Vec::new();
        loop {
            let bytes_read = stdin
                .lock()
                .read_until(b'\n', &mut buf)
                .expect("Failed to read from stdin");

            // EOF
            if bytes_read == 0 {
                break;
            }

            let stdin_request = RunCommandRequest {
                stdin: Some(replace(&mut buf, Vec::new())),
                ..Default::default()
            };
            block_on(tx.send(stdin_request)).expect("Failed to send stdin packet");
        }
    });

    let mut resp_stream = client
        .run_command(ReceiverStream::new(rx))
        .await
        .expect("Failed to start run_command rpc")
        .into_inner();

    let mut stdout_writer = io::stdout();
    let mut stderr_writer = io::stderr();
    let mut exit_code = None;
    while let Some(resp) = resp_stream.next().await {
        let resp = resp.expect("Error while running command");

        if let Some(ref stdout) = resp.stdout {
            stdout_writer.write_all(stdout).await.unwrap();
            stdout_writer.flush().await.unwrap();
        }

        if let Some(ref stderr) = resp.stderr {
            stderr_writer.write_all(stderr).await.unwrap();
            stderr_writer.flush().await.unwrap();
        }

        if resp.exit_code.is_some() {
            exit_code = resp.exit_code;
        }
    }

    let Some(exit_code) = exit_code else {
        panic!("Stream ended with no exit code");
    };
    let Ok(exit_code) = u8::try_from(exit_code) else {
        panic!("Stream ended with invalid exit code (must be u8): {exit_code}")
    };

    println!("[flasherd-client] Terminated naturally with status code {exit_code}");
    exit(exit_code.into());
}
