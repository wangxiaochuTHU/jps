#[cfg(feature = "decomp")]
pub mod Decomp {
    use itertools::Itertools;
    pub use yakf::kf::{self, so3::SO3, Grp3, Vec3};
    // pub use yakf::linalg::*;
    pub struct Ellipsoid {
        /// center of the ellipsoid
        center: Vec3,
        /// rotation matrix of the ellipsoid
        r: SO3,
    }
    pub fn find_perpendicular_direction(n1: &Vec3) -> Vec3 {
        let x = n1;
        let x = n1.abs();
        // find a coordinate i-th whose value is not 0
        let (i, v_i) = n1.iter().enumerate().find(|x| *x.1 != 0.0).unwrap();
        // find another coordinate j-th, s.t.  j != i
        let (j, v_j) = n1.iter().enumerate().find(|x| x.0 != i).unwrap();
        // make a vector n, such that:  n_i = v_j, n_j = -v_i, n_k = 0
        let mut n2 = Vec3::zeros();
        n2[i] = *v_j;
        n2[j] = -*v_i;
        // normalize
        n2 = n2.normalize();
        // return
        n2
    }
    pub fn find_third_direction(n1: &Vec3, n2: &Vec3) -> Vec3 {
        // n1 × n2, input should be unit vectors.
        n1.cross(n2)
    }
    pub fn get_attitude_from_one_vector(n1: &Vec3) -> Grp3 {
        let n2 = find_perpendicular_direction(n1);
        let n3 = find_third_direction(n1, &n2);
        let attitude_matrix = Grp3::from_columns(&[*n1, n2, n3]);
        attitude_matrix
    }
}
