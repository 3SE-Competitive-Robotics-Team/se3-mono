use std::path::PathBuf;
use std::thread;
use std::time::{Duration, Instant};

use clap::{Parser, ValueEnum};
use control_core::{SenderConfig, SenderMode, build_sender};
use locomotion_core::protocol::PolicyTargetFrame;

#[derive(Debug, Parser)]
#[command(about = "Run the SE3 control loop.")]
struct Args {
    #[arg(long, value_enum, default_value_t = SenderArg::Usb)]
    sender: SenderArg,

    #[arg(long = "socket-path", default_value = control_core::DEFAULT_SIM_SOCKET_PATH)]
    socket_path: PathBuf,

    #[arg(long = "usb-port", default_value = "auto")]
    usb_port: String,

    #[arg(long, default_value_t = 921600)]
    baudrate: i32,

    #[arg(long = "write-timeout-s", default_value_t = 0.02)]
    write_timeout_s: f64,

    #[arg(long = "rate-hz", default_value_t = 50.0)]
    rate_hz: f64,

    #[arg(long = "max-steps", default_value_t = 0)]
    max_steps: u32,

    #[arg(long = "print-every", default_value_t = 50)]
    print_every: u32,
}

#[derive(Debug, Clone, Copy, ValueEnum)]
enum SenderArg {
    Usb,
    SimSocket,
    Both,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    let cfg = SenderConfig {
        mode: match args.sender {
            SenderArg::Usb => SenderMode::Usb,
            SenderArg::SimSocket => SenderMode::SimSocket,
            SenderArg::Both => SenderMode::Both,
        },
        usb_port: args.usb_port,
        baudrate: args.baudrate,
        write_timeout_s: args.write_timeout_s,
        socket_path: args.socket_path,
    };
    let mut sender = build_sender(&cfg)?;
    let period = Duration::from_secs_f64(1.0 / args.rate_hz.max(1.0));
    let mut next_tick = Instant::now();
    let mut seq = 0_u32;

    loop {
        if args.max_steps > 0 && seq >= args.max_steps {
            break;
        }
        let target = sample_target(seq);
        sender.send(&target)?;
        if args.print_every > 0 && seq.is_multiple_of(args.print_every) {
            println!(
                "control sent seq={} joint_pos={:?} wheel_vel={:?}",
                target.seq, target.joint_pos, target.wheel_vel
            );
        }
        seq = seq.wrapping_add(1);
        next_tick += period;
        let now = Instant::now();
        if next_tick > now {
            thread::sleep(next_tick - now);
        } else {
            next_tick = now;
        }
    }

    Ok(())
}

fn sample_target(seq: u32) -> PolicyTargetFrame {
    let phase = seq as f32 * 0.02;
    PolicyTargetFrame {
        seq,
        joint_pos: [
            0.4610 + 0.02 * phase.sin(),
            0.4742 + 0.02 * phase.cos(),
            0.4610 - 0.02 * phase.sin(),
            0.4742 - 0.02 * phase.cos(),
        ],
        wheel_vel: [0.0, 0.0],
    }
}
