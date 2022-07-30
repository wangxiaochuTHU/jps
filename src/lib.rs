extern crate alloc;
extern crate itertools;
extern crate nalgebra as na;
/// Re-export nalgebra
pub mod linalg {
    pub use na::base::*;
    pub use na::RealField;
}
pub mod base;
pub mod base_with_obstacle_input;

pub use base::{jps_3d_v1, Dir, Grid};
pub use base_with_obstacle_input::jps_3d_v2;
mod constant;
