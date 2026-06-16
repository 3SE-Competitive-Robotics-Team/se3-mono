use crate::rbt_base::rbt_geometry::rbt_point2::RbtImgPoint2;
use crate::rbt_mod::rbt_estimator::rbt_enemy_dynamic_model::EnemyArmorType;

/// 作为 Detector 的输出和 Solver 的输入
#[derive(Debug, Clone)]
pub struct DetectedArmor {
    key_points: [RbtImgPoint2; 5],
    _id: usize, // 当前帧画面唯一 id，用于区分每一块装甲板
    armor_type: EnemyArmorType,
    neutral_color: bool,
}

impl DetectedArmor {
    pub fn new(
        center: RbtImgPoint2,
        lt: RbtImgPoint2,
        lb: RbtImgPoint2,
        rb: RbtImgPoint2,
        rt: RbtImgPoint2,
        id: usize,
    ) -> Self {
        Self::with_type_and_color(center, lt, lb, rb, rt, id, EnemyArmorType::Small, false)
    }

    pub fn with_type_and_color(
        center: RbtImgPoint2,
        lt: RbtImgPoint2,
        lb: RbtImgPoint2,
        rb: RbtImgPoint2,
        rt: RbtImgPoint2,
        id: usize,
        armor_type: EnemyArmorType,
        neutral_color: bool,
    ) -> Self {
        DetectedArmor {
            key_points: [center, lt, lb, rb, rt],
            _id: id,
            armor_type,
            neutral_color,
        }
    }

    /// 根据五点坐标来创建
    pub fn from_corner_coords(corner: &[f32; 10], id: usize) -> Self {
        DetectedArmor {
            key_points: [
                RbtImgPoint2::new_screen_pixel(corner[0], corner[1]),
                RbtImgPoint2::new_screen_pixel(corner[2], corner[3]),
                RbtImgPoint2::new_screen_pixel(corner[4], corner[5]),
                RbtImgPoint2::new_screen_pixel(corner[6], corner[7]),
                RbtImgPoint2::new_screen_pixel(corner[8], corner[9]),
            ],
            _id: id,
            armor_type: EnemyArmorType::Small,
            neutral_color: false,
        }
    }

    #[inline(always)]
    pub fn center(&self) -> RbtImgPoint2 {
        self.key_points[0]
    }

    #[inline(always)]
    pub fn lt(&self) -> RbtImgPoint2 {
        self.key_points[1]
    }

    #[inline(always)]
    pub fn lb(&self) -> RbtImgPoint2 {
        self.key_points[2]
    }

    #[inline(always)]
    pub fn rb(&self) -> RbtImgPoint2 {
        self.key_points[3]
    }

    #[inline(always)]
    pub fn rt(&self) -> RbtImgPoint2 {
        self.key_points[4]
    }

    pub fn corner_points(&self) -> [RbtImgPoint2; 4] {
        [self.lt(), self.lb(), self.rb(), self.rt()]
    }

    pub fn armor_type(&self) -> EnemyArmorType {
        self.armor_type
    }

    pub fn neutral_color(&self) -> bool {
        self.neutral_color
    }
}
