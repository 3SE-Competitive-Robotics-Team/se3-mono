use std::path::{Path, PathBuf};

use ndarray as nd;
use ort::{
    inputs,
    session::{Session, builder::GraphOptimizationLevel},
    value::{Outlet, TensorRef},
};
use thiserror::Error;

use crate::ort_ep::{OrtEpError, OrtExecutionProvider, configure_session_builder};

#[derive(Debug, Error)]
pub enum OrtPolicyError {
    #[error("checkpoint not found: {0}")]
    CheckpointNotFound(PathBuf),
    #[error("{0}")]
    OrtEp(#[from] OrtEpError),
    #[error("{0}")]
    Ort(#[from] ort::Error),
    #[error("{0}")]
    OrtBuilder(String),
    #[error("unsupported ONNX policy rnn type: {0}")]
    UnsupportedRnn(String),
    #[error("{name} metadata is invalid: {value}")]
    InvalidMetadata { name: &'static str, value: String },
    #[error("obs shape mismatch: expected {expected}, got {got}")]
    ObsShapeMismatch { expected: usize, got: usize },
    #[error("{name} shape mismatch: expected {expected:?}, got {got:?}")]
    ShapeMismatch {
        name: &'static str,
        expected: Vec<usize>,
        got: Vec<usize>,
    },
}

pub struct OrtPolicyRuntime {
    pub checkpoint_path: PathBuf,
    pub execution_provider: OrtExecutionProvider,
    pub iteration: String,
    pub num_obs: usize,
    pub num_actions: usize,
    pub activation: String,
    pub rnn_type: String,
    pub rnn_hidden_dim: usize,
    pub rnn_num_layers: usize,
    session: Session,
    obs: nd::Array2<f32>,
    hidden: nd::Array3<f32>,
}

impl OrtPolicyRuntime {
    pub fn new(checkpoint: impl AsRef<Path>, configured_ep: &str) -> Result<Self, OrtPolicyError> {
        let checkpoint_path = checkpoint.as_ref().to_path_buf();
        if !checkpoint_path.exists() {
            return Err(OrtPolicyError::CheckpointNotFound(checkpoint_path));
        }
        let engine_cache_path = checkpoint_path
            .parent()
            .unwrap_or_else(|| Path::new("."))
            .join("ort_engine_cache");
        std::fs::create_dir_all(&engine_cache_path).ok();

        let session_builder = Session::builder()?;
        let (session_builder, execution_provider) =
            configure_session_builder(session_builder, configured_ep, &engine_cache_path)?;
        let session = session_builder
            .with_optimization_level(GraphOptimizationLevel::Level3)
            .map_err(builder_error)?
            .with_intra_threads(1)
            .map_err(builder_error)?
            .with_inter_threads(1)
            .map_err(builder_error)?
            .commit_from_file(&checkpoint_path)?;

        let metadata = session.metadata()?;
        let rnn_type = metadata
            .custom("rnn_type")
            .unwrap_or_else(|| "gru".to_string());
        if rnn_type != "gru" {
            return Err(OrtPolicyError::UnsupportedRnn(rnn_type));
        }
        let num_obs = metadata_usize(&metadata, "num_obs")
            .unwrap_or_else(|| outlet_last_dim(session.inputs(), "obs").unwrap_or(0));
        let num_actions = metadata_usize(&metadata, "num_actions")
            .unwrap_or_else(|| outlet_last_dim(session.outputs(), "action").unwrap_or(0));
        let rnn_hidden_dim = metadata_usize(&metadata, "rnn_hidden_dim")
            .unwrap_or_else(|| outlet_last_dim(session.inputs(), "hidden_in").unwrap_or(0));
        let rnn_num_layers = metadata_usize(&metadata, "rnn_num_layers")
            .unwrap_or_else(|| outlet_dim(session.inputs(), "hidden_in", 0).unwrap_or(0));
        let iteration = metadata
            .custom("iteration")
            .unwrap_or_else(|| "unknown".to_string());
        let activation = metadata
            .custom("activation")
            .unwrap_or_else(|| "unknown".to_string());
        drop(metadata);

        let runtime = Self {
            checkpoint_path,
            execution_provider,
            iteration,
            num_obs,
            num_actions,
            activation,
            rnn_type,
            rnn_hidden_dim,
            rnn_num_layers,
            session,
            obs: nd::Array2::zeros((1, num_obs)),
            hidden: nd::Array3::zeros((rnn_num_layers, 1, rnn_hidden_dim)),
        };
        runtime.validate_model_io()?;
        Ok(runtime)
    }

    pub fn reset(&mut self) {
        self.hidden.fill(0.0);
    }

    pub fn act(&mut self, obs: &[f32]) -> Result<Vec<f32>, OrtPolicyError> {
        if obs.len() != self.num_obs {
            return Err(OrtPolicyError::ObsShapeMismatch {
                expected: self.num_obs,
                got: obs.len(),
            });
        }
        for (dst, src) in self.obs.iter_mut().zip(obs.iter().copied()) {
            *dst = src;
        }
        let outputs = self.session.run(inputs![
            "obs" => TensorRef::from_array_view(self.obs.view())?,
            "hidden_in" => TensorRef::from_array_view(self.hidden.view())?,
        ])?;

        let action = outputs["action"].try_extract_array::<f32>()?;
        let action_shape = action.shape().to_vec();
        if action_shape != [1, self.num_actions] {
            return Err(OrtPolicyError::ShapeMismatch {
                name: "action",
                expected: vec![1, self.num_actions],
                got: action_shape,
            });
        }
        let out = action.iter().copied().collect::<Vec<_>>();

        let hidden = outputs["hidden_out"].try_extract_array::<f32>()?;
        let hidden_shape = hidden.shape().to_vec();
        let expected_hidden = vec![self.rnn_num_layers, 1, self.rnn_hidden_dim];
        if hidden_shape != expected_hidden {
            return Err(OrtPolicyError::ShapeMismatch {
                name: "hidden_out",
                expected: expected_hidden,
                got: hidden_shape,
            });
        }
        for (dst, src) in self.hidden.iter_mut().zip(hidden.iter().copied()) {
            *dst = src;
        }
        Ok(out)
    }

    pub fn policy_type(&self) -> String {
        format!(
            "onnx-gru(hidden={}, layers={})",
            self.rnn_hidden_dim, self.rnn_num_layers
        )
    }

    fn validate_model_io(&self) -> Result<(), OrtPolicyError> {
        expect_outlet_shape(self.session.inputs(), "obs", &[1, self.num_obs], "obs")?;
        expect_outlet_shape(
            self.session.inputs(),
            "hidden_in",
            &[self.rnn_num_layers, 1, self.rnn_hidden_dim],
            "hidden_in",
        )?;
        expect_outlet_shape(
            self.session.outputs(),
            "action",
            &[1, self.num_actions],
            "action",
        )?;
        expect_outlet_shape(
            self.session.outputs(),
            "hidden_out",
            &[self.rnn_num_layers, 1, self.rnn_hidden_dim],
            "hidden_out",
        )?;
        Ok(())
    }
}

fn metadata_usize(metadata: &ort::session::ModelMetadata<'_>, name: &'static str) -> Option<usize> {
    metadata.custom(name)?.parse().ok()
}

fn outlet_shape(outlets: &[Outlet], name: &str) -> Option<Vec<usize>> {
    let outlet = outlets.iter().find(|outlet| outlet.name() == name)?;
    let shape = outlet.dtype().tensor_shape()?;
    let mut out = Vec::with_capacity(shape.len());
    for dim in shape.iter().copied() {
        if dim < 0 {
            return None;
        }
        out.push(dim as usize);
    }
    Some(out)
}

fn outlet_dim(outlets: &[Outlet], name: &str, index: usize) -> Option<usize> {
    outlet_shape(outlets, name)?.get(index).copied()
}

fn outlet_last_dim(outlets: &[Outlet], name: &str) -> Option<usize> {
    outlet_shape(outlets, name)?.last().copied()
}

fn expect_outlet_shape(
    outlets: &[Outlet],
    outlet_name: &'static str,
    expected: &[usize],
    error_name: &'static str,
) -> Result<(), OrtPolicyError> {
    let got = outlet_shape(outlets, outlet_name).unwrap_or_default();
    if got != expected {
        return Err(OrtPolicyError::ShapeMismatch {
            name: error_name,
            expected: expected.to_vec(),
            got,
        });
    }
    Ok(())
}

fn builder_error(err: ort::Error<ort::session::builder::SessionBuilder>) -> OrtPolicyError {
    OrtPolicyError::OrtBuilder(err.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ort_runtime_matches_npz_reference_first_two_actions() {
        let Ok(root) = std::env::var("SE3_RECOVERY_CHECKPOINT") else {
            eprintln!("skipping ORT model test: SE3_RECOVERY_CHECKPOINT is not set");
            return;
        };
        let mut policy = OrtPolicyRuntime::new(root, "cpu").unwrap();
        let mut obs = [0.0_f32; 32];
        obs[5] = -1.0;
        obs[10] = 1.1;

        let first = policy.act(&obs).unwrap();
        assert_close(
            &first,
            &[
                0.044738493859767914,
                -1.5334395170211792,
                0.13665412366390228,
                -0.8563693761825562,
                -1.435538649559021,
                0.699100911617279,
            ],
            2.0e-5,
        );

        let second = policy.act(&obs).unwrap();
        assert_close(
            &second,
            &[
                0.010364338755607605,
                -1.9548662900924683,
                0.07704169303178787,
                -1.171495795249939,
                -1.921726942062378,
                1.0233880281448364,
            ],
            2.0e-5,
        );
    }

    fn assert_close(actual: &[f32], expected: &[f32], tol: f32) {
        assert_eq!(actual.len(), expected.len());
        for (idx, (actual, expected)) in actual.iter().zip(expected).enumerate() {
            let diff = (actual - expected).abs();
            assert!(
                diff <= tol,
                "idx={idx} actual={actual} expected={expected} diff={diff} tol={tol}"
            );
        }
    }
}
