//! 敌方单位模型定义模块
//!
//! 该模块定义了RoboMaster比赛中敌方单位（装甲板）的模型和相关数据结构。
//! 包括装甲板类型、敌方ID、阵营等基本信息。
//!
//! 主要组件：
//! - EnemyId: 敌方单位标识枚举
//! - EnemyArmorType: 装甲板大小类型

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// 描述敌方装甲板大或者小
pub enum EnemyArmorType {
    Small,
    Large,
}

/// 用于描述装甲板/敌方车辆的唯一标记型 ID
#[derive(Debug, Clone, Copy, Eq, PartialEq, Hash, strum::Display)]
pub enum EnemyId {
    Hero1,
    Engineer2,
    Infantry3,
    Infantry4,
    Infantry5,
    Sentry7,
    Outpost8,
    Invalid,
}

/// 描述敌方阵营
#[derive(Debug, Clone, Copy, PartialEq, Eq, strum::Display)]
pub enum EnemyFaction {
    R,
    B,
}
