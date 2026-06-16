//! SerialLeg locomotion deployment runtime and policy semantics.

pub mod action_delay;
pub mod cdc;
pub mod fourbar;
pub mod height_default;
pub mod motor;
pub mod observation_config;
pub mod ort_ep;
pub mod ort_policy;
pub mod policy_io;
pub mod protocol;
pub mod recovery_observation;
pub mod recovery_runtime;
pub mod replay_telemetry;
pub mod robot;
pub mod visualize_cdc_state;

pub use action_delay::{ActionDelayConfig, DelayResampleMode, delay_seconds_to_steps};
pub use fourbar::{
    FOURBAR_SURROGATE_MARKER, active_angle_from_output_knee, is_fourbar_surrogate_name_set,
    output_knee_from_active_angle, output_knee_jacobian, output_to_policy_pos,
    output_to_policy_vel, policy_to_output_pos, policy_to_output_torque, policy_to_output_vel,
};
pub use height_default::policy_default_from_height;
pub use motor::{DM8009P, M3508_C620_14, M3508_HEXROLL, MotorSpec};
pub use observation_config::ObservationConfig;
pub use policy_io::{
    DecodedPolicyAction, PolicyActionDecoder, PolicyObservationResult, build_policy_observation,
};
pub use robot::{Joint, JointGroup, RobotConfig, Termination};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::policy_io::PolicyActionDecoderConfig;

    #[test]
    fn fourbar_matches_python_reference() {
        let cfg = RobotConfig::default();
        let alphas = [0.0, 0.37, cfg.active_rod_angle_limits[1]];
        let knees = alphas.map(output_knee_from_active_angle);
        assert_close64(
            &knees,
            &[0.0, -0.3809985977567711, -1.389388135291841],
            2.0e-6,
        );

        let policy = [
            cfg.default_dof_pos[0],
            cfg.default_dof_pos[1],
            cfg.default_dof_pos[2],
            cfg.default_dof_pos[3],
        ];
        let output = policy_to_output_pos(policy);
        assert_close64(
            &output,
            &[
                -0.275422946189,
                -1.2422634109042463,
                0.275422946189,
                1.2422634109042463,
            ],
            2.0e-6,
        );
        assert_close64(&output_to_policy_pos(output), &policy, 2.0e-6);
        let jacobian = alphas.map(|alpha| output_knee_jacobian(alpha, false));
        assert_close64(
            &jacobian,
            &[
                -1.0893378740064463,
                -0.9879831596758722,
                -0.7157458738880862,
            ],
            2.0e-6,
        );
    }

    #[test]
    fn height_default_matches_python_reference() {
        let values = [
            policy_default_from_height(0.16, None),
            policy_default_from_height(0.22, None),
            policy_default_from_height(0.28, None),
        ];
        assert_close64(
            &values[0],
            &[
                -0.07103051464637974,
                -1.5805657846962748,
                0.07103051464637974,
                1.5805657846962748,
            ],
            2.0e-6,
        );
        assert_close64(
            &values[1],
            &[
                -0.23798194922700222,
                -1.5504238879326389,
                0.23798194922700222,
                1.5504238879326389,
            ],
            2.0e-6,
        );
        assert_close64(
            &values[2],
            &[
                -0.4903575884677846,
                -1.3781786642792317,
                0.4903575884677846,
                1.3781786642792317,
            ],
            2.0e-6,
        );
    }

    #[test]
    fn action_decoder_matches_python_reference() {
        let decoder = PolicyActionDecoder::new(PolicyActionDecoderConfig {
            height_conditioned_action_default: true,
            ..PolicyActionDecoderConfig::default()
        });
        let decoded = decoder
            .decode([0.2, -0.3, 0.4, -0.5, 0.6, -0.7], Some(0.22), None, None)
            .unwrap();
        assert_close32(
            &decoded.leg_target,
            &[
                0.390_336_6,
                -0.695_675,
                1.494_619,
                2.429_677,
            ],
            2.0e-5,
        );
        assert_close32(&decoded.wheel_vel_target, &[27.000_002, -31.5], 2.0e-5);
    }

    fn assert_close64<const N: usize>(actual: &[f64; N], expected: &[f64; N], tol: f64) {
        for (idx, (a, e)) in actual.iter().zip(expected).enumerate() {
            assert!((*a - *e).abs() <= tol, "idx {idx}: actual {a} expected {e}");
        }
    }

    fn assert_close32<const N: usize>(actual: &[f32], expected: &[f32; N], tol: f32) {
        assert_eq!(actual.len(), expected.len());
        for (idx, (a, e)) in actual.iter().zip(expected).enumerate() {
            assert!((*a - *e).abs() <= tol, "idx {idx}: actual {a} expected {e}");
        }
    }
}
