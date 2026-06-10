# Repository Instructions

This repository is a Rust monorepo for RoboMaster robot runtime code. It is still a scaffold, so do not claim working crates, builds, tests, deployment, or runtime behavior until those pieces exist.

## Layout

- `apps/`: runnable process crates, such as `control`, `auto_strike`, and future robot processes.
- `crates/`: shared Rust libraries and process-independent logic.
- `drivers/`: hardware-specific Rust crates and protocol adapters.
- `platforms/`: build, cross-compilation, runtime, and deployment settings for compute platforms.
- `robots/`: per-robot runtime configuration and deployment files.
- `docs/`: design notes and project documentation.

## Conventions

- Keep robot-specific parameters in configuration, not Cargo features.
- Keep compute-platform settings in `platforms/`, not in robot configs or driver names.
- Use Cargo features only for compile-time capabilities or optional dependencies.
- Keep code split by responsibility, not by robot model, unless there is a real hardware or strategy boundary.
- Use Chinese for user-facing project docs. Use English for agent and tooling instructions.
