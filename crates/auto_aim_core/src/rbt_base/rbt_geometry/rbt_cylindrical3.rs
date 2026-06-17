#[derive(Debug, Clone)]
pub struct RbtCylindricalPoint3 {
    pub rho: f64,     // 相对与世界坐标系原点的距离
    pub theta_d: f64, // 在世界坐标系下的角度
    pub z: f64,
}

impl RbtCylindricalPoint3 {
    pub fn new(dist: f64, angle_yaw_d: f64, z: f64) -> Self {
        Self {
            rho: dist,
            theta_d: angle_yaw_d,
            z,
        }
    }

    pub fn from_xyz(xyz: impl Into<na::Point3<f64>>) -> Self {
        let p = xyz.into();
        let (x, y, z) = (p.x, p.y, p.z);
        let rho = (x * x + y * y).sqrt();
        let theta_d = y.atan2(x).to_degrees();
        Self::new(rho, theta_d, z)
    }

    pub fn to_xyz(&self) -> na::Point3<f64> {
        let theta = self.theta_d.to_radians();
        na::Point3::new(self.rho * theta.cos(), self.rho * theta.sin(), self.z)
    }
}
