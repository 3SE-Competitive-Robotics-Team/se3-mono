use std::sync::OnceLock;

use crate::fourbar::output_knee_from_active_angle;
use crate::robot::RobotConfig;

const LUT_SIZE: usize = 1024;
const WHEEL_RADIUS: f64 = 0.06;
const BASE_COM_X: f64 = -0.01780372;
const LF1_BODY_XZ: [f64; 2] = [-0.12990117, 0.04639203];
const LF1_JOINT_XZ: [f64; 2] = [-0.05003347, -0.04149627];
const WHEEL_BODY_XZ: [f64; 2] = [-0.15699, -0.21049];

struct HeightDefaultLut {
    active_by_length: Vec<f64>,
    length_grid: Vec<f64>,
    active_grid: Vec<f64>,
    vec_x_grid: Vec<f64>,
    vec_z_grid: Vec<f64>,
}

static HEIGHT_LUT: OnceLock<HeightDefaultLut> = OnceLock::new();

pub fn policy_default_from_height(command_height: f64, cfg: Option<&RobotConfig>) -> [f64; 4] {
    let robot_cfg;
    let cfg = match cfg {
        Some(cfg) => cfg,
        None => {
            robot_cfg = RobotConfig::default();
            &robot_cfg
        }
    };
    let _ = cfg;
    let lut = height_default_lut();
    let target_x = BASE_COM_X;
    let target_z = WHEEL_RADIUS - command_height;
    let target_length = target_x.hypot(target_z).clamp(
        lut.length_grid[0],
        lut.length_grid[lut.length_grid.len() - 1],
    );
    let active = interp_monotonic(target_length, &lut.length_grid, &lut.active_by_length);
    let vec_x = interp_monotonic(active, &lut.active_grid, &lut.vec_x_grid);
    let vec_z = interp_monotonic(active, &lut.active_grid, &lut.vec_z_grid);
    let lf = vec_x.atan2(-vec_z) - target_x.atan2(-target_z);
    let rf = -lf;
    let lb = lf - active;
    let rb = rf + active;
    [lf, lb, rf, rb]
}

fn height_default_lut() -> &'static HeightDefaultLut {
    HEIGHT_LUT.get_or_init(|| {
        let cfg = RobotConfig::default();
        let [lower, upper] = cfg.active_rod_angle_limits;
        let active_grid: Vec<f64> = (0..LUT_SIZE)
            .map(|idx| lower + (upper - lower) * idx as f64 / (LUT_SIZE - 1) as f64)
            .collect();
        let mut vec_x_grid = Vec::with_capacity(LUT_SIZE);
        let mut vec_z_grid = Vec::with_capacity(LUT_SIZE);
        let mut length_pairs = Vec::with_capacity(LUT_SIZE);
        for (idx, active) in active_grid.iter().copied().enumerate() {
            let output_knee = output_knee_from_active_angle(active);
            let (vec_x, vec_z) = leg_vector(output_knee);
            vec_x_grid.push(vec_x);
            vec_z_grid.push(vec_z);
            length_pairs.push(((vec_x * vec_x + vec_z * vec_z).sqrt(), idx));
        }
        length_pairs.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut active_by_length = Vec::with_capacity(LUT_SIZE);
        let mut length_grid = Vec::with_capacity(LUT_SIZE);
        for (length, idx) in length_pairs {
            length_grid.push(length);
            active_by_length.push(active_grid[idx]);
        }
        HeightDefaultLut {
            active_by_length,
            length_grid,
            active_grid,
            vec_x_grid,
            vec_z_grid,
        }
    })
}

fn leg_vector(output_knee: f64) -> (f64, f64) {
    let body = LF1_BODY_XZ;
    let joint = LF1_JOINT_XZ;
    let wheel = WHEEL_BODY_XZ;
    let cos_q = output_knee.cos();
    let sin_q = output_knee.sin();
    let rot_joint_x = cos_q * joint[0] + sin_q * joint[1];
    let rot_joint_z = -sin_q * joint[0] + cos_q * joint[1];
    let rot_wheel_x = cos_q * wheel[0] + sin_q * wheel[1];
    let rot_wheel_z = -sin_q * wheel[0] + cos_q * wheel[1];
    let x = body[0] + joint[0] - rot_joint_x + rot_wheel_x;
    let z = body[1] + joint[1] - rot_joint_z + rot_wheel_z;
    (x, z)
}

fn interp_monotonic(x: f64, xp: &[f64], fp: &[f64]) -> f64 {
    let x_clamped = x.clamp(xp[0], xp[xp.len() - 1]);
    match xp.binary_search_by(|probe| probe.total_cmp(&x_clamped)) {
        Ok(idx) => fp[idx],
        Err(idx_hi) => {
            let idx_hi = idx_hi.clamp(1, xp.len() - 1);
            let idx_lo = idx_hi - 1;
            let x0 = xp[idx_lo];
            let x1 = xp[idx_hi];
            let y0 = fp[idx_lo];
            let y1 = fp[idx_hi];
            let t = (x_clamped - x0) / (x1 - x0).max(1.0e-12);
            y0 + t * (y1 - y0)
        }
    }
}
