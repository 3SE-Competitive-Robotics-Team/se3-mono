//! 状态估计器模块
//!
//! 该模块实现了基于 YPD 角度 tracker 的敌方单位状态估计功能。
//! 通过融合视觉测量数据和几何运动模型，对敌方单位的位置、速度等状态进行估计和预测。
//!
//! 主要组件：
//! - EstimatorStateMachine: 估计器状态机，管理估计器的不同工作状态
//! - RbtEstimator: 单个敌方单位的状态估计器实现
//! - RbtHandlerPoll: 所有敌方单位估计器的管理池
//!

use std::collections::HashMap;
use std::time::Instant;

use crate::rbt_infra::rbt_cfg::EstimatorCfg;
use crate::rbt_mod::rbt_armor::solved_armor::SolvedArmor;
use crate::rbt_mod::rbt_solver::{RbtSolvedResult, RbtSolvedResults};

use rbt_enemy_dynamic_model::EnemyId;
use rbt_enemy_select::{EnemySelectHandler, TRACKED_ENEMY_IDS};
use rbt_estimator_state::EstimatorStateMachine;
use rbt_ypd_angle_tracker::{YpdAngleTracker, YpdObservation, YpdTrackerSnapshot};

const SPIN_YAW_RATE_THRESHOLD_RAD_S: f64 = 0.1;
const SPIN_COUNT_THRESHOLD: usize = 10;
const TRANSLATION_SPEED_THRESHOLD_MPS: f64 = 0.2;
const TRANSLATION_COUNT_THRESHOLD: usize = 5;
const OBSERVATION_JUMP_THRESHOLD_M: f64 = 0.15;
const OUTPOST_AIMING_CX_PX: f64 = -160.0;
const OUTPOST_AIMING_CY_PX: f64 = -300.0;

/// 敌方单位基础模型
pub mod rbt_enemy_dynamic_model;
mod rbt_enemy_select;
pub mod rbt_ypd_angle_tracker;

/// Snapshot exported by the estimator for the fire-control planner.
///
/// Units are intentionally aligned with `vivsionn::Target`: meters and radians.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TargetMotionState {
    Static,
    Translation,
    Spinning,
    Transpin,
}

impl TargetMotionState {
    pub fn is_static(self) -> bool {
        self == TargetMotionState::Static
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EnemyTrackSnapshot {
    pub enemy_id: EnemyId,
    pub armor_count: usize,
    pub center_xy_m: na::Point2<f64>,
    pub center_velocity_xy_mps: na::Vector2<f64>,
    pub armor_z_m: f64,
    pub armor_z_velocity_mps: f64,
    pub body_yaw_rad: f64,
    pub body_yaw_rate_rad_s: f64,
    pub primary_radius_m: f64,
    pub secondary_radius_delta_m: f64,
    pub height_delta_m: f64,
    pub state_age_s: f64,
    pub track_valid: bool,
    pub fire_permit: bool,
    pub motion_state: TargetMotionState,
    pub motion_uniform: bool,
    pub observation_stable: bool,
    pub motion_translation_burst_metric: f64,
    pub motion_translation_drift_metric: f64,
    pub motion_yaw_accel_metric: f64,
}

impl EnemyTrackSnapshot {
    pub fn planner_state(&self) -> [f64; 11] {
        [
            self.center_xy_m.x,
            self.center_velocity_xy_mps.x,
            self.center_xy_m.y,
            self.center_velocity_xy_mps.y,
            self.armor_z_m,
            self.armor_z_velocity_mps,
            self.body_yaw_rad,
            self.body_yaw_rate_rad_s,
            self.primary_radius_m,
            self.secondary_radius_delta_m,
            self.height_delta_m,
        ]
    }
}

pub mod rbt_estimator_state {
    use crate::rbt_infra::rbt_cfg::EstimatorCfg;

    use crate::rbt_mod::rbt_solver::RbtSolvedResult;

    /// 顶层状态机
    #[derive(Debug, Clone, PartialEq, strum::Display)]
    pub enum EstimatorStateMachine {
        Init, // 初始化
        Sleep,
        WakeUp, // 从睡眠中恢复
        Track,  // 跟踪状态
        Lost {
            // 目标丢失（未识别，装甲板灭）
            time_stamp: tokio::time::Instant, // 丢失时间戳
        },
        Recovery, // 从丢失状态中恢复
    }

    impl EstimatorStateMachine {
        pub fn update(&mut self, solved_enemy: &Option<RbtSolvedResult>, cfg: &EstimatorCfg) {
            use EstimatorStateMachine::*;
            match self {
                Init => {
                    // 只会在初始化用到，然后在第一次 update 流转至其他状态
                    *self = match solved_enemy {
                        Some(_) => WakeUp,
                        None => Sleep,
                    }
                }
                Sleep => {
                    // 看到装甲板则唤醒估计器
                    if solved_enemy.is_some() {
                        *self = WakeUp;
                    }
                    // 没看到就继续休眠
                }
                WakeUp => {
                    // 看到装甲板则进入追踪
                    *self = match solved_enemy {
                        Some(_) => Track,
                        None => Lost {
                            time_stamp: tokio::time::Instant::now(),
                        },
                    }
                }
                Track => {
                    // 如果solved_enemy 是 None 进入Lost状态，并记录当前时间戳
                    if solved_enemy.is_none() {
                        *self = Lost {
                            time_stamp: tokio::time::Instant::now(),
                        };
                    }
                }
                Lost { time_stamp } => {
                    *self = match (
                        solved_enemy.is_some(),                             // 是否检测到装甲板
                        time_stamp.elapsed() > cfg.lost_wait_duration_ms(), // 是否超时
                    ) {
                        (true, _) => Recovery,  // 如果检测到装甲板，进入Recovery状态
                        (false, true) => Sleep, // 如果没检测到装甲板且超时，进入Sleep状态
                        (false, false) => Lost {
                            time_stamp: *time_stamp, /* copy */
                        }, // 如果没检测到装甲板且未超时，保持Lost状态
                    };
                }
                Recovery => {
                    *self = match solved_enemy {
                        Some(_) => Track,
                        None => Lost {
                            time_stamp: tokio::time::Instant::now(),
                        },
                    }
                }
            }
        }
    }
}

/// 状态估计器
#[derive(Debug, Clone)]
pub struct RbtEstimator {
    state: EstimatorStateMachine,
    ypd_angle_tracker: YpdAngleTracker,
    latest_tracker_snapshot: Option<YpdTrackerSnapshot>,
    last_update_tp: Option<Instant>,
    fire_observation_hold_frames: usize,
    last_observed_armor_position_m: Option<na::Point3<f64>>,
    tracked_armor_id: Option<EnemyId>,
    tracked_armor_type: Option<rbt_enemy_dynamic_model::EnemyArmorType>,
    tracked_neutral_color_frames: usize,
    spin_count: usize,
    translation_count: usize,
    motion_state: TargetMotionState,
    pub enemy_id: EnemyId,
    pub fire: bool,             // 当前是否开火
    pub single_or_double: bool, // 当前帧是否有多装甲板观测
}

impl RbtEstimator {
    pub fn new(enemy_id: EnemyId) -> Self {
        Self {
            state: EstimatorStateMachine::Init,
            ypd_angle_tracker: YpdAngleTracker::new(),
            latest_tracker_snapshot: None,
            last_update_tp: None,
            fire_observation_hold_frames: 0,
            last_observed_armor_position_m: None,
            tracked_armor_id: None,
            tracked_armor_type: None,
            tracked_neutral_color_frames: 0,
            spin_count: 0,
            translation_count: 0,
            motion_state: TargetMotionState::Static,
            enemy_id,
            fire: false,
            single_or_double: false,
        }
    }

    pub fn update(&mut self, cfg: &EstimatorCfg, solved_enemy: &Option<RbtSolvedResult>) {
        let dt_s = self.update_dt_s();
        let solved_enemy = self.filter_neutral_solution(cfg, solved_enemy);

        self.state.update(&solved_enemy, cfg);

        if matches!(
            self.state,
            EstimatorStateMachine::Init | EstimatorStateMachine::Sleep
        ) {
            self.ypd_angle_tracker.reset();
            self.latest_tracker_snapshot = None;
            self.reset_fire_observation_hold();
        }

        self.update_global_vars(&solved_enemy);
        let tracker_was_initialized = self.ypd_angle_tracker.is_initialized();
        self.update_tracker(cfg, solved_enemy.as_ref(), dt_s, tracker_was_initialized);
        self.update_fire_observation_hold(cfg, solved_enemy.as_ref(), tracker_was_initialized);
        self.update_motion_state();
    }

    pub fn update_with_frame(
        &mut self,
        cfg: &EstimatorCfg,
        selected_solution: &Option<RbtSolvedResult>,
        solved_enemies: &RbtSolvedResults,
    ) {
        let matched_solution = selected_solution
            .as_ref()
            .map(|solution| self.match_solution_for_current_target(cfg, solution, solved_enemies));
        self.update(cfg, &matched_solution);
    }

    pub fn snapshot(&self, _cfg: &EstimatorCfg) -> Option<EnemyTrackSnapshot> {
        if matches!(
            self.state,
            EstimatorStateMachine::Init | EstimatorStateMachine::Sleep
        ) {
            return None;
        }
        let tracker_snapshot = self.tracker_snapshot()?;
        let state = tracker_snapshot.state11d;
        let state_age_s = match self.state {
            EstimatorStateMachine::Lost { time_stamp } => time_stamp.elapsed().as_secs_f64(),
            _ => 0.0,
        };

        Some(EnemyTrackSnapshot {
            enemy_id: self.enemy_id,
            armor_count: tracker_snapshot.armor_num,
            center_xy_m: na::Point2::new(state[0] * 0.001, state[2] * 0.001),
            center_velocity_xy_mps: na::Vector2::new(state[1] * 0.001, state[3] * 0.001),
            armor_z_m: state[4] * 0.001,
            armor_z_velocity_mps: state[5] * 0.001,
            body_yaw_rad: state[6],
            body_yaw_rate_rad_s: state[7],
            primary_radius_m: state[8] * 0.001,
            secondary_radius_delta_m: state[9] * 0.001,
            height_delta_m: state[10] * 0.001,
            state_age_s,
            track_valid: !tracker_snapshot.diverged,
            fire_permit: self.fire,
            motion_state: self.motion_state,
            motion_uniform: motion_uniform(tracker_snapshot),
            observation_stable: observation_stable(tracker_snapshot)
                && self.fire_observation_hold_frames == 0,
            motion_translation_burst_metric: metric_mm_to_m(
                tracker_snapshot.motion_translation_burst_metric,
            ),
            motion_translation_drift_metric: metric_mm_to_m(
                tracker_snapshot.motion_translation_drift_metric,
            ),
            motion_yaw_accel_metric: tracker_snapshot.motion_yaw_accel_metric,
        })
    }

    fn update_global_vars(&mut self, solved_enemy: &Option<RbtSolvedResult>) {
        // 设置fire
        self.fire = matches!(self.state, EstimatorStateMachine::Track);

        // 设置single_or_double
        self.single_or_double = solved_enemy
            .as_ref()
            .map(|s| s.armors.len() > 1)
            .unwrap_or(false);

        if let Some(solved) = solved_enemy
            && let Some(primary) = solved.armors.first()
        {
            if primary.neutral_color() {
                self.tracked_neutral_color_frames =
                    self.tracked_neutral_color_frames.saturating_add(1);
            } else {
                self.tracked_neutral_color_frames = 0;
            }
        }
    }

    fn filter_neutral_solution(
        &mut self,
        cfg: &EstimatorCfg,
        solved_enemy: &Option<RbtSolvedResult>,
    ) -> Option<RbtSolvedResult> {
        let Some(solved) = solved_enemy else {
            self.tracked_neutral_color_frames = 0;
            return None;
        };
        let neutral_primary = solved
            .armors
            .first()
            .is_some_and(|armor| armor.neutral_color());
        if neutral_primary && self.tracked_neutral_color_frames >= cfg.armor_neutral_grace_frames {
            None
        } else {
            Some(solved.clone())
        }
    }

    fn update_fire_observation_hold(
        &mut self,
        cfg: &EstimatorCfg,
        solved_enemy: Option<&RbtSolvedResult>,
        tracker_was_initialized: bool,
    ) {
        let Some(solved_enemy) = solved_enemy else {
            self.reset_fire_observation_hold();
            return;
        };

        let primary_observation_m = primary_armor_observation_position_m(solved_enemy);
        let observation_jump =
            tracker_was_initialized && self.observation_jump_for_solution(solved_enemy);
        if cfg.fire_block_on_armor_jump && observation_jump {
            self.fire_observation_hold_frames = self
                .fire_observation_hold_frames
                .max(cfg.fire_armor_jump_block_frames);
        } else if self.fire_observation_hold_frames > 0 {
            self.fire_observation_hold_frames -= 1;
        }
        self.last_observed_armor_position_m = primary_observation_m;
    }

    fn reset_fire_observation_hold(&mut self) {
        self.fire_observation_hold_frames = 0;
        self.last_observed_armor_position_m = None;
        self.tracked_armor_id = None;
        self.tracked_armor_type = None;
        self.tracked_neutral_color_frames = 0;
        self.spin_count = 0;
        self.translation_count = 0;
        self.motion_state = TargetMotionState::Static;
    }

    pub fn tracker_snapshot(&self) -> Option<&YpdTrackerSnapshot> {
        self.latest_tracker_snapshot.as_ref()
    }

    fn update_dt_s(&mut self) -> f64 {
        let now = Instant::now();
        let dt_s = self
            .last_update_tp
            .map(|last| now.duration_since(last).as_secs_f64())
            .unwrap_or(0.01);
        self.last_update_tp = Some(now);
        dt_s.clamp(0.001, 0.05)
    }

    fn update_tracker(
        &mut self,
        cfg: &EstimatorCfg,
        solved_enemy: Option<&RbtSolvedResult>,
        dt_s: f64,
        tracker_was_initialized: bool,
    ) {
        use EstimatorStateMachine::*;

        match &self.state {
            Init | Sleep => {}
            WakeUp | Recovery | Track => {
                self.predict_or_reset_tracker(dt_s);
                if let Some(solved) = solved_enemy {
                    self.correct_tracker_with_solution(cfg, solved, tracker_was_initialized);
                }
                self.sync_tracker_snapshot();
            }
            Lost { .. } => {
                self.predict_or_reset_tracker(dt_s);
                self.sync_tracker_snapshot();
            }
        }
    }

    fn predict_or_reset_tracker(&mut self, dt_s: f64) {
        if self.ypd_angle_tracker.diverged() || self.ypd_angle_tracker.bad_convergence() {
            self.ypd_angle_tracker.reset();
            self.latest_tracker_snapshot = None;
            return;
        }
        self.ypd_angle_tracker.predict(dt_s);
    }

    fn correct_tracker_with_solution(
        &mut self,
        cfg: &EstimatorCfg,
        solved: &RbtSolvedResult,
        tracker_was_initialized: bool,
    ) {
        let armor_num = armor_num_for_enemy(self.enemy_id);
        let observations = self.ypd_observations(solved);
        let Some(preferred_index) = preferred_observation_index(&observations, cfg, armor_num)
        else {
            return;
        };

        if !self.ypd_angle_tracker.is_initialized() {
            self.ypd_angle_tracker
                .init(&observations[preferred_index], armor_num);
        } else {
            self.ypd_angle_tracker.note_observation_jump(
                tracker_was_initialized && self.observation_jump_for_solution(solved),
                cfg,
            );
            self.ypd_angle_tracker
                .update_batch(&observations, Some(preferred_index), cfg);
        }
        if let Some(primary) = solved.armors.get(preferred_index) {
            self.tracked_armor_id = Some(primary.armor_id());
            self.tracked_armor_type = Some(primary.armor_type());
        }
    }

    fn observation_jump_for_solution(&self, solved_enemy: &RbtSolvedResult) -> bool {
        primary_armor_observation_position_m(solved_enemy)
            .zip(self.last_observed_armor_position_m)
            .is_some_and(|(current, last)| (current - last).norm() > OBSERVATION_JUMP_THRESHOLD_M)
    }

    fn update_motion_state(&mut self) {
        let Some(snapshot) = self.latest_tracker_snapshot.as_ref() else {
            self.motion_state = TargetMotionState::Static;
            return;
        };
        let state = snapshot.state11d;
        let translation_speed_mps = (state[1] * 0.001).hypot(state[3] * 0.001);
        let yaw_rate_rad_s = state[7].abs();

        if yaw_rate_rad_s < SPIN_YAW_RATE_THRESHOLD_RAD_S {
            self.spin_count = 0;
        } else {
            self.spin_count = self.spin_count.saturating_add(1);
        }

        if self.spin_count > SPIN_COUNT_THRESHOLD {
            self.motion_state = if translation_speed_mps > TRANSLATION_SPEED_THRESHOLD_MPS {
                TargetMotionState::Transpin
            } else {
                TargetMotionState::Spinning
            };
            return;
        }

        if translation_speed_mps >= TRANSLATION_SPEED_THRESHOLD_MPS {
            self.translation_count = self.translation_count.saturating_add(1);
        } else {
            self.translation_count = 0;
        }
        self.motion_state = if self.translation_count >= TRANSLATION_COUNT_THRESHOLD {
            TargetMotionState::Translation
        } else {
            TargetMotionState::Static
        };
    }

    fn sync_tracker_snapshot(&mut self) {
        self.latest_tracker_snapshot = self.ypd_angle_tracker.snapshot();
    }

    fn match_solution_for_current_target(
        &self,
        cfg: &EstimatorCfg,
        selected_solution: &RbtSolvedResult,
        solved_enemies: &RbtSolvedResults,
    ) -> RbtSolvedResult {
        let tracked_id = self.tracked_armor_id;
        let tracked_type = self.tracked_armor_type;
        let mut armors = self.sorted_visible_armors(cfg, solved_enemies);
        armors.retain(|armor| {
            tracked_type.is_none_or(|tracked_type| armor.armor_type() == tracked_type)
                && (cfg.ignore_same_number_condition_switch
                    || tracked_id.is_none_or(|tracked_id| armor.armor_id() == tracked_id))
        });

        if armors.is_empty() {
            return selected_solution.clone();
        }

        RbtSolvedResult {
            coord: selected_solution.coord.clone(),
            armors,
        }
    }

    fn sorted_visible_armors(
        &self,
        cfg: &EstimatorCfg,
        solved_enemies: &RbtSolvedResults,
    ) -> Vec<SolvedArmor> {
        let mut armors = Vec::new();
        for solution in solved_enemies.values().flatten() {
            armors.extend(solution.armors.iter().cloned());
        }
        armors.sort_by(|lhs, rhs| armor_observation_cmp(lhs, rhs, cfg));
        armors
    }

    fn ypd_observations(&self, solved: &RbtSolvedResult) -> Vec<YpdObservation> {
        let center = solved.coord.to_xy();
        let armor_num = armor_num_for_enemy(self.enemy_id);
        let sign = tracker_radial_sign(armor_num);

        solved
            .armors
            .iter()
            .map(|armor| {
                let armor_center = armor.enemy_center_xy().unwrap_or(center);
                let position_vec = armor.pose().translation.vector;
                let position = na::Point3::new(position_vec.x, position_vec.y, position_vec.z);
                let dx = position.x - armor_center.x;
                let dy = position.y - armor_center.y;
                let radius_from_center = dx.hypot(dy);
                let radius_hint = if armor.radius().is_finite() && armor.radius() > 1e-6 {
                    armor.radius()
                } else {
                    radius_from_center
                };
                let yaw_rad = if armor_num == 3 {
                    armor.observed_yaw_rad()
                } else if radius_from_center > 1e-6 {
                    (dy / sign).atan2(dx / sign)
                } else {
                    armor.observed_yaw_rad()
                };
                let image_center = armor.center();

                YpdObservation {
                    position_mm: position,
                    yaw_rad,
                    image_center: na::Point2::new(image_center.x, image_center.y),
                    radius_hint_mm: radius_hint,
                }
            })
            .collect()
    }
}

fn armor_num_for_enemy(enemy_id: EnemyId) -> usize {
    if enemy_id == EnemyId::Outpost8 { 3 } else { 4 }
}

fn armor_observation_cmp(
    lhs: &SolvedArmor,
    rhs: &SolvedArmor,
    cfg: &EstimatorCfg,
) -> std::cmp::Ordering {
    let lhs_center = lhs.center();
    let rhs_center = rhs.center();
    let lhs_distance = squared_image_distance(
        lhs_center.x,
        lhs_center.y,
        cfg.image_center_x,
        cfg.image_center_y,
    );
    let rhs_distance = squared_image_distance(
        rhs_center.x,
        rhs_center.y,
        cfg.image_center_x,
        cfg.image_center_y,
    );

    lhs_distance
        .total_cmp(&rhs_distance)
        .then_with(|| {
            armor_number_sort_key(lhs.armor_id()).cmp(&armor_number_sort_key(rhs.armor_id()))
        })
        .then_with(|| {
            armor_type_sort_key(lhs.armor_type()).cmp(&armor_type_sort_key(rhs.armor_type()))
        })
        .then_with(|| lhs_center.x.total_cmp(&rhs_center.x))
        .then_with(|| lhs_center.y.total_cmp(&rhs_center.y))
}

fn squared_image_distance(x: f64, y: f64, center_x: f64, center_y: f64) -> f64 {
    let dx = x - center_x;
    let dy = y - center_y;
    dx * dx + dy * dy
}

fn armor_number_sort_key(enemy_id: EnemyId) -> usize {
    match enemy_id {
        EnemyId::Hero1 => 1,
        EnemyId::Engineer2 => 2,
        EnemyId::Infantry3 => 3,
        EnemyId::Infantry4 => 4,
        EnemyId::Infantry5 => 5,
        EnemyId::Sentry7 => 7,
        EnemyId::Outpost8 => 9,
        EnemyId::Invalid => usize::MAX,
    }
}

fn armor_type_sort_key(armor_type: rbt_enemy_dynamic_model::EnemyArmorType) -> usize {
    match armor_type {
        rbt_enemy_dynamic_model::EnemyArmorType::Small => 0,
        rbt_enemy_dynamic_model::EnemyArmorType::Large => 1,
    }
}

fn tracker_radial_sign(armor_num: usize) -> f64 {
    if armor_num == 3 { 1.0 } else { -1.0 }
}

fn preferred_observation_index(
    observations: &[YpdObservation],
    cfg: &EstimatorCfg,
    armor_num: usize,
) -> Option<usize> {
    observations
        .iter()
        .enumerate()
        .min_by(|(_, lhs), (_, rhs)| {
            image_center_score(lhs, cfg, armor_num)
                .total_cmp(&image_center_score(rhs, cfg, armor_num))
        })
        .map(|(index, _)| index)
}

fn image_center_score(observation: &YpdObservation, cfg: &EstimatorCfg, armor_num: usize) -> f64 {
    let (center_x, center_y) = if armor_num == 3 {
        (
            cfg.image_center_x - OUTPOST_AIMING_CX_PX,
            cfg.image_center_y - OUTPOST_AIMING_CY_PX,
        )
    } else {
        (cfg.image_center_x, cfg.image_center_y)
    };
    let dx = observation.image_center.x - center_x;
    let dy = observation.image_center.y - center_y;
    dx * dx + dy * dy
}

fn primary_armor_observation_position_m(solved_enemy: &RbtSolvedResult) -> Option<na::Point3<f64>> {
    solved_enemy.armors.first().map(|armor| {
        let position = armor.pose().translation.vector;
        na::Point3::new(position.x * 0.001, position.y * 0.001, position.z * 0.001)
    })
}

fn observation_stable(snapshot: &YpdTrackerSnapshot) -> bool {
    !snapshot.diverged && (snapshot.converged || snapshot.recent_nis_failures <= 1)
}

fn motion_uniform(snapshot: &YpdTrackerSnapshot) -> bool {
    metric_under(snapshot.motion_translation_burst_metric, 8_000.0)
        && metric_under(snapshot.motion_translation_drift_metric, 4_000.0)
        && metric_under(snapshot.motion_yaw_accel_metric, 20.0)
}

fn metric_under(value: f64, threshold: f64) -> bool {
    !value.is_finite() || value.abs() < threshold
}

fn metric_mm_to_m(value: f64) -> f64 {
    if value.is_finite() {
        value * 0.001
    } else {
        f64::NAN
    }
}

/// 管理所有敌方单位的估计器。
#[derive(Debug, Clone)]
pub struct RbtHandlerPoll {
    estimators: HashMap<EnemyId, RbtEstimator>,
    enemy_selector: EnemySelectHandler,
}

impl RbtHandlerPoll {
    pub fn new() -> Self {
        let mut estimators = HashMap::with_capacity(6);
        for enemy_id in TRACKED_ENEMY_IDS {
            estimators.insert(enemy_id, RbtEstimator::new(enemy_id));
        }

        Self {
            estimators,
            enemy_selector: EnemySelectHandler::default(),
        }
    }

    pub fn update(&mut self, cfg: &EstimatorCfg, solved_enemies: RbtSolvedResults) {
        let selected_enemy_id = self.enemy_selector.select(cfg, &solved_enemies);
        let no_solution = None;

        for enemy_id in TRACKED_ENEMY_IDS {
            let solved_enemy = if selected_enemy_id == Some(enemy_id) {
                solved_enemies.get(&enemy_id).unwrap_or(&no_solution)
            } else {
                &no_solution
            };

            self.estimators
                .entry(enemy_id)
                .or_insert_with(|| RbtEstimator::new(enemy_id))
                .update_with_frame(cfg, solved_enemy, &solved_enemies);
        }
    }

    pub fn selected_enemy_id(&self) -> Option<EnemyId> {
        self.enemy_selector.selected_enemy_id()
    }

    pub fn selected_snapshot(&self, cfg: &EstimatorCfg) -> Option<EnemyTrackSnapshot> {
        let enemy_id = self.selected_enemy_id()?;
        self.estimators.get(&enemy_id)?.snapshot(cfg)
    }
}

impl Default for RbtHandlerPoll {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;
    use crate::rbt_base::rbt_geometry::rbt_cylindrical2::RbtCylindricalPoint2;
    use crate::rbt_base::rbt_geometry::rbt_point2::RbtImgPoint2;
    use crate::rbt_mod::rbt_armor::detected_armor::DetectedArmor;
    use crate::rbt_mod::rbt_armor::detected_armor::DetectedArmorMeta;
    use crate::rbt_mod::rbt_armor::solved_armor::SolvedArmor;
    use na::Isometry3;

    fn estimator_cfg(enemy_lost_wait_duration_ms: u64) -> EstimatorCfg {
        toml::from_str(&format!(
            "\
armor_lost_wait_duration_ms = 100
enemy_lost_wait_duration_ms = {enemy_lost_wait_duration_ms}
fire_block_on_armor_jump = true
fire_armor_jump_block_frames = 3
"
        ))
        .unwrap()
    }

    fn solved_enemy(center_x: f32, center_y: f32) -> RbtSolvedResult {
        let detected_armor = DetectedArmor::new(
            RbtImgPoint2::new_screen_pixel(center_x, center_y),
            RbtImgPoint2::new_screen_pixel(center_x - 10.0, center_y - 5.0),
            RbtImgPoint2::new_screen_pixel(center_x - 10.0, center_y + 5.0),
            RbtImgPoint2::new_screen_pixel(center_x + 10.0, center_y + 5.0),
            RbtImgPoint2::new_screen_pixel(center_x + 10.0, center_y - 5.0),
            DetectedArmorMeta::small(0, EnemyId::Hero1),
        );

        RbtSolvedResult {
            coord: RbtCylindricalPoint2::new(1_000.0, 0.0),
            armors: vec![SolvedArmor::new(
                detected_armor,
                Isometry3::identity(),
                0.0,
                0.0,
                200.0,
            )],
        }
    }

    fn solved_enemy_with_armors(centers: &[(f32, f32)]) -> RbtSolvedResult {
        let mut armors = Vec::with_capacity(centers.len());
        for (idx, (center_x, center_y)) in centers.iter().copied().enumerate() {
            let detected_armor = DetectedArmor::new(
                RbtImgPoint2::new_screen_pixel(center_x, center_y),
                RbtImgPoint2::new_screen_pixel(center_x - 10.0, center_y - 5.0),
                RbtImgPoint2::new_screen_pixel(center_x - 10.0, center_y + 5.0),
                RbtImgPoint2::new_screen_pixel(center_x + 10.0, center_y + 5.0),
                RbtImgPoint2::new_screen_pixel(center_x + 10.0, center_y - 5.0),
                DetectedArmorMeta::small(idx, EnemyId::Hero1),
            );
            let pose = Isometry3::translation(200.0 + idx as f64 * 20.0, idx as f64 * 200.0, 100.0);
            armors.push(SolvedArmor::new(
                detected_armor,
                pose,
                idx as f64 * 90.0,
                0.0,
                200.0,
            ));
        }

        RbtSolvedResult {
            coord: RbtCylindricalPoint2::new(1_000.0, 0.0),
            armors,
        }
    }

    fn frame(targets: &[(EnemyId, (f32, f32))]) -> RbtSolvedResults {
        let mut solved_enemies = RbtSolvedResults::default();
        for (enemy_id, (x, y)) in targets {
            solved_enemies.insert(*enemy_id, Some(solved_enemy(*x, *y)));
        }
        solved_enemies
    }

    fn stable_tracker_snapshot() -> YpdTrackerSnapshot {
        let mut state11d = [0.0; 11];
        state11d[8] = 200.0;

        YpdTrackerSnapshot {
            state11d,
            state9: [0.0; 9],
            tracked_id: 0,
            armor_num: 4,
            tracked_armor_xyza: [200.0, 0.0, 0.0, 0.0],
            predicted_armors_xyza: Vec::new(),
            last_nis: 0.0,
            converged: true,
            diverged: false,
            recent_nis_failures: 0,
            motion_translation_burst_metric: 0.0,
            motion_translation_drift_metric: 0.0,
            motion_yaw_accel_metric: 0.0,
        }
    }

    fn estimator_with_stable_snapshot() -> RbtEstimator {
        let mut estimator = RbtEstimator::new(EnemyId::Hero1);
        estimator.state = EstimatorStateMachine::Track;
        estimator.fire = true;
        estimator.latest_tracker_snapshot = Some(stable_tracker_snapshot());
        estimator
    }

    #[test]
    fn handler_poll_feeds_only_the_selected_estimator() {
        let cfg = estimator_cfg(1_000);
        let mut handler_poll = RbtHandlerPoll::new();

        handler_poll.update(
            &cfg,
            frame(&[
                (EnemyId::Hero1, (320.0, 192.0)),
                (EnemyId::Infantry3, (321.0, 192.0)),
            ]),
        );

        assert_eq!(handler_poll.selected_enemy_id(), Some(EnemyId::Hero1));
        assert!(
            handler_poll.estimators[&EnemyId::Hero1]
                .tracker_snapshot()
                .is_some()
        );
        assert!(
            handler_poll.estimators[&EnemyId::Infantry3]
                .tracker_snapshot()
                .is_none()
        );
    }

    #[test]
    fn handler_poll_does_not_switch_while_selected_enemy_is_visible() {
        let cfg = estimator_cfg(1_000);
        let mut handler_poll = RbtHandlerPoll::new();

        handler_poll.update(
            &cfg,
            frame(&[
                (EnemyId::Hero1, (320.0, 192.0)),
                (EnemyId::Infantry3, (321.0, 192.0)),
            ]),
        );
        handler_poll.update(
            &cfg,
            frame(&[
                (EnemyId::Hero1, (600.0, 192.0)),
                (EnemyId::Infantry3, (320.0, 192.0)),
            ]),
        );

        assert_eq!(handler_poll.selected_enemy_id(), Some(EnemyId::Hero1));
        assert!(
            handler_poll.estimators[&EnemyId::Hero1]
                .tracker_snapshot()
                .is_some()
        );
        assert!(
            handler_poll.estimators[&EnemyId::Infantry3]
                .tracker_snapshot()
                .is_none()
        );
    }

    #[test]
    fn selected_snapshot_exports_selected_enemy_in_planner_units() {
        let cfg = estimator_cfg(1_000);
        let mut handler_poll = RbtHandlerPoll::new();

        handler_poll.update(&cfg, frame(&[(EnemyId::Hero1, (320.0, 192.0))]));
        handler_poll.update(&cfg, frame(&[(EnemyId::Hero1, (320.0, 192.0))]));

        let snapshot = handler_poll.selected_snapshot(&cfg).unwrap();
        assert_eq!(snapshot.enemy_id, EnemyId::Hero1);
        assert_eq!(snapshot.armor_count, 4);
        assert!(snapshot.track_valid);
        assert!(snapshot.fire_permit);
        assert_eq!(snapshot.motion_state, TargetMotionState::Static);
        assert!(snapshot.motion_uniform);
        assert!(snapshot.observation_stable);
        assert!((snapshot.center_xy_m.x - 0.2).abs() < 1e-9);
        assert!(snapshot.center_xy_m.y.abs() < 1e-9);
        assert!(snapshot.body_yaw_rad.abs() < 1e-9);
        assert!((snapshot.primary_radius_m - 0.2).abs() < 1e-9);
        assert_eq!(snapshot.planner_state()[0], snapshot.center_xy_m.x);
    }

    #[test]
    fn selected_snapshot_is_none_without_target() {
        let cfg = estimator_cfg(1_000);
        let mut handler_poll = RbtHandlerPoll::new();

        handler_poll.update(&cfg, RbtSolvedResults::default());

        assert!(handler_poll.selected_snapshot(&cfg).is_none());
    }

    #[test]
    fn fire_hold_blocks_stable_tracker_observation() {
        let cfg = estimator_cfg(1_000);
        let mut estimator = estimator_with_stable_snapshot();

        assert!(estimator.snapshot(&cfg).unwrap().observation_stable);

        estimator.fire_observation_hold_frames = 1;

        assert!(!estimator.snapshot(&cfg).unwrap().observation_stable);
    }

    #[test]
    fn armor_jump_starts_and_releases_fire_hold_for_configured_frames() {
        let cfg = estimator_cfg(1_000);
        let mut estimator = RbtEstimator::new(EnemyId::Hero1);
        let single = Some(solved_enemy(320.0, 192.0));
        let jumped = Some(solved_enemy_with_armors(&[(320.0, 192.0), (520.0, 192.0)]));

        estimator.update(&cfg, &single);
        estimator.update(&cfg, &single);
        assert_eq!(estimator.fire_observation_hold_frames, 0);

        estimator.update(&cfg, &jumped);
        assert_eq!(estimator.fire_observation_hold_frames, 3);

        estimator.update(&cfg, &jumped);
        assert_eq!(estimator.fire_observation_hold_frames, 2);
        estimator.update(&cfg, &jumped);
        assert_eq!(estimator.fire_observation_hold_frames, 1);
        estimator.update(&cfg, &jumped);
        assert_eq!(estimator.fire_observation_hold_frames, 0);
    }

    #[test]
    fn armor_jump_fire_block_can_be_disabled() {
        let mut cfg = estimator_cfg(1_000);
        cfg.fire_block_on_armor_jump = false;
        let mut estimator = RbtEstimator::new(EnemyId::Hero1);
        let single = Some(solved_enemy(320.0, 192.0));
        let jumped = Some(solved_enemy_with_armors(&[(320.0, 192.0), (520.0, 192.0)]));

        estimator.update(&cfg, &single);
        estimator.update(&cfg, &single);
        estimator.update(&cfg, &jumped);

        assert_eq!(estimator.fire_observation_hold_frames, 0);
    }

    #[test]
    fn no_target_resets_armor_jump_fire_hold() {
        let cfg = estimator_cfg(1_000);
        let mut estimator = RbtEstimator::new(EnemyId::Hero1);
        let single = Some(solved_enemy(320.0, 192.0));
        let jumped = Some(solved_enemy_with_armors(&[(320.0, 192.0), (520.0, 192.0)]));

        estimator.update(&cfg, &single);
        estimator.update(&cfg, &single);
        estimator.update(&cfg, &jumped);
        assert!(estimator.fire_observation_hold_frames > 0);

        estimator.update(&cfg, &None);

        assert_eq!(estimator.fire_observation_hold_frames, 0);
        assert!(estimator.last_observed_armor_position_m.is_none());
    }
}
