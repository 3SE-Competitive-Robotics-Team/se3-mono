use std::f64::consts::PI;
use std::sync::OnceLock;

pub const FOURBAR_SURROGATE_MARKER: &str = "fourbar_surrogate_marker";

const KNEE_X: f64 = -0.17993464;
const KNEE_Z: f64 = 0.00489576;
const DRIVE_X: f64 = 0.04009536;
const DRIVE_Z: f64 = 0.04530576;
const COUPLER_LEN: f64 = 0.17000000057221706;
const CALF_LEN: f64 = 0.06500221953251904;
const CALF_ZERO_ANGLE: f64 = 0.6923948261163193;
const ACTIVE_LOWER: f64 = 0.0;
const ACTIVE_UPPER: f64 = 1.5095352700498952;
const LUT_SIZE: usize = 8192;

struct FourbarLut {
    alpha_grid: Vec<f64>,
    knee_grid: Vec<f64>,
    inverse_knee_grid: Vec<f64>,
    inverse_alpha_grid: Vec<f64>,
    jacobian_grid: Vec<f64>,
}

static FOURBAR_LUT: OnceLock<FourbarLut> = OnceLock::new();

pub fn is_fourbar_surrogate_name_set(site_names: &[impl AsRef<str>]) -> bool {
    site_names
        .iter()
        .any(|name| name.as_ref() == FOURBAR_SURROGATE_MARKER)
}

pub fn output_knee_from_active_angle(active_angle: f64) -> f64 {
    let lut = fourbar_lut();
    interp_lut(active_angle, &lut.alpha_grid, &lut.knee_grid)
}

pub fn policy_to_output_pos(policy_pos: [f64; 4]) -> [f64; 4] {
    let left_alpha = (policy_pos[0] - policy_pos[1]).clamp(ACTIVE_LOWER, ACTIVE_UPPER);
    let right_alpha = (policy_pos[3] - policy_pos[2]).clamp(ACTIVE_LOWER, ACTIVE_UPPER);
    [
        policy_pos[0],
        output_knee_from_active_angle(left_alpha),
        policy_pos[2],
        -output_knee_from_active_angle(right_alpha),
    ]
}

pub fn output_to_policy_pos(output_pos: [f64; 4]) -> [f64; 4] {
    let left_alpha = active_angle_from_output_knee(output_pos[1], false);
    let right_alpha = active_angle_from_output_knee(output_pos[3], true);
    [
        output_pos[0],
        output_pos[0] - left_alpha,
        output_pos[2],
        output_pos[2] + right_alpha,
    ]
}

pub fn output_to_policy_vel(output_pos: [f64; 4], output_vel: [f64; 4]) -> [f64; 4] {
    let left_alpha = active_angle_from_output_knee(output_pos[1], false);
    let right_alpha = active_angle_from_output_knee(output_pos[3], true);
    let left_j = output_knee_jacobian(left_alpha, false);
    let right_j = output_knee_jacobian(right_alpha, true);
    let left_alpha_dot = output_vel[1] / safe_denominator(left_j);
    let right_alpha_dot = output_vel[3] / safe_denominator(right_j);
    [
        output_vel[0],
        output_vel[0] - left_alpha_dot,
        output_vel[2],
        output_vel[2] + right_alpha_dot,
    ]
}

pub fn policy_to_output_vel(policy_pos: [f64; 4], policy_vel: [f64; 4]) -> [f64; 4] {
    let left_alpha = (policy_pos[0] - policy_pos[1]).clamp(ACTIVE_LOWER, ACTIVE_UPPER);
    let right_alpha = (policy_pos[3] - policy_pos[2]).clamp(ACTIVE_LOWER, ACTIVE_UPPER);
    let left_j = output_knee_jacobian(left_alpha, false);
    let right_j = output_knee_jacobian(right_alpha, true);
    [
        policy_vel[0],
        left_j * (policy_vel[0] - policy_vel[1]),
        policy_vel[2],
        right_j * (policy_vel[3] - policy_vel[2]),
    ]
}

pub fn active_angle_from_output_knee(output_knee: f64, right_side: bool) -> f64 {
    let target = if right_side {
        -output_knee
    } else {
        output_knee
    };
    let lut = fourbar_lut();
    interp_lut(target, &lut.inverse_knee_grid, &lut.inverse_alpha_grid)
}

pub fn output_knee_jacobian(active_angle: f64, right_side: bool) -> f64 {
    let lut = fourbar_lut();
    let value = interp_lut(active_angle, &lut.alpha_grid, &lut.jacobian_grid);
    if right_side { -value } else { value }
}

pub fn policy_to_output_torque(policy_pos: [f64; 4], policy_torque: [f64; 4]) -> [f64; 4] {
    let left_alpha = (policy_pos[0] - policy_pos[1]).clamp(ACTIVE_LOWER, ACTIVE_UPPER);
    let right_alpha = (policy_pos[3] - policy_pos[2]).clamp(ACTIVE_LOWER, ACTIVE_UPPER);
    let left_j = output_knee_jacobian(left_alpha, false);
    let right_j = output_knee_jacobian(right_alpha, true);
    [
        policy_torque[0] + policy_torque[1],
        -policy_torque[1] / safe_denominator(left_j),
        policy_torque[2] + policy_torque[3],
        policy_torque[3] / safe_denominator(right_j),
    ]
}

fn fourbar_lut() -> &'static FourbarLut {
    FOURBAR_LUT.get_or_init(|| {
        let alpha_grid: Vec<f64> = (0..LUT_SIZE)
            .map(|idx| {
                ACTIVE_LOWER + (ACTIVE_UPPER - ACTIVE_LOWER) * idx as f64 / (LUT_SIZE - 1) as f64
            })
            .collect();
        let knee_grid: Vec<f64> = alpha_grid
            .iter()
            .copied()
            .map(output_knee_from_active_angle_analytic)
            .collect();
        let (inverse_knee_grid, inverse_alpha_grid) = inverse_lut_grids(&knee_grid, &alpha_grid);
        let eps = 1.0e-3;
        let jacobian_grid: Vec<f64> = alpha_grid
            .iter()
            .copied()
            .map(|alpha| {
                let lo = (alpha - eps).clamp(ACTIVE_LOWER, ACTIVE_UPPER);
                let hi = (alpha + eps).clamp(ACTIVE_LOWER, ACTIVE_UPPER);
                (output_knee_from_active_angle_analytic(hi)
                    - output_knee_from_active_angle_analytic(lo))
                    / (hi - lo).max(1.0e-6)
            })
            .collect();

        FourbarLut {
            alpha_grid,
            knee_grid,
            inverse_knee_grid,
            inverse_alpha_grid,
            jacobian_grid,
        }
    })
}

fn output_knee_from_active_angle_analytic(active_angle: f64) -> f64 {
    let alpha = active_angle.clamp(ACTIVE_LOWER, ACTIVE_UPPER);
    let beta = -alpha;
    let cos_b = beta.cos();
    let sin_b = beta.sin();
    let px = cos_b * DRIVE_X + sin_b * DRIVE_Z;
    let pz = -sin_b * DRIVE_X + cos_b * DRIVE_Z;

    let dx = px - KNEE_X;
    let dz = pz - KNEE_Z;
    let dist = (dx * dx + dz * dz).max(1.0e-12).sqrt();
    let ex = dx / dist;
    let ez = dz / dist;

    let along = (CALF_LEN.powi(2) - COUPLER_LEN.powi(2) + dist.powi(2)) / (2.0 * dist);
    let height = (CALF_LEN.powi(2) - along.powi(2)).max(0.0).sqrt();
    let cx = KNEE_X + along * ex - height * ez;
    let cz = KNEE_Z + along * ez + height * ex;

    let phi = (cz - KNEE_Z).atan2(cx - KNEE_X);
    wrap_angle(CALF_ZERO_ANGLE - phi)
}

#[allow(clippy::panic)]
fn inverse_lut_grids(knee_grid: &[f64], alpha_grid: &[f64]) -> (Vec<f64>, Vec<f64>) {
    let increasing = knee_grid.windows(2).all(|pair| pair[1] >= pair[0]);
    let decreasing = knee_grid.windows(2).all(|pair| pair[1] <= pair[0]);
    if increasing {
        return (knee_grid.to_vec(), alpha_grid.to_vec());
    }
    if decreasing {
        let mut knee = knee_grid.to_vec();
        let mut alpha = alpha_grid.to_vec();
        knee.reverse();
        alpha.reverse();
        return (knee, alpha);
    }
    panic!("fourbar LUT output knee grid is not monotonic");
}

fn interp_lut(query: f64, x_grid: &[f64], y_grid: &[f64]) -> f64 {
    let q = query.clamp(x_grid[0], x_grid[x_grid.len() - 1]);
    match x_grid.binary_search_by(|probe| probe.total_cmp(&q)) {
        Ok(idx) => y_grid[idx],
        Err(idx_hi) => {
            let idx_hi = idx_hi.clamp(1, x_grid.len() - 1);
            let idx_lo = idx_hi - 1;
            let x0 = x_grid[idx_lo];
            let x1 = x_grid[idx_hi];
            let y0 = y_grid[idx_lo];
            let y1 = y_grid[idx_hi];
            let weight = (q - x0) / (x1 - x0).max(f64::EPSILON);
            y0 + weight * (y1 - y0)
        }
    }
}

fn wrap_angle(angle: f64) -> f64 {
    (angle + PI).rem_euclid(2.0 * PI) - PI
}

fn safe_denominator(value: f64) -> f64 {
    if value.abs() < 1.0e-6 {
        if value < 0.0 { -1.0e-6 } else { 1.0e-6 }
    } else {
        value
    }
}
