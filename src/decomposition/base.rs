#[cfg(feature = "decomp")]
#[allow(non_upper_case_globals)]
#[allow(unused)]
pub mod decomp_tool {
    use itertools::Itertools;
    pub use yakf::kf::{self, so3::SO3, Grp3, Vec3};
    // pub use yakf::linalg::*;
    pub const a_max: f64 = 10.0;
    pub const v_max: f64 = 5.0;
    pub const r_robot: f64 = 0.2;

    #[derive(Debug)]
    pub struct Decomp {
        // /// radius of robot
        // pub r_r: f64,
        /// radius of safe flight,  r_s = v_max^2/(2*a_max)
        pub r_s: f64,

        /// line is defined by two points
        pub line_ends: [Vec3; 2],

        /// boundingbox
        pub bbox: BoundingBox,

        /// ellipsoid
        pub ellipsoid: Ellipsoid,
    }
    impl Decomp {
        pub fn init(p1: &Vec3, p2: &Vec3) -> Self {
            let r_s = 0.5 * v_max.powi(2) / a_max;
            let d = r_s + r_robot;
            let center = 0.5 * (p1 + p2);
            let line = p2 - p1;
            let a = line.norm();
            let b = a;
            let c = a;
            let n1 = find_bbox_dir_x(&line);
            let n2 = find_bbox_dir_y(&n1);
            let n3 = find_bbox_dir_z(&n1, &n2);
            let hplanes: [HyperPlane; 6] = [
                HyperPlane::new(n1, &center, &d),
                HyperPlane::new(-n1, &center, &d),
                HyperPlane::new(n2, &center, &d),
                HyperPlane::new(-n2, &center, &d),
                HyperPlane::new(n3, &center, &d),
                HyperPlane::new(-n3, &center, &d),
            ];
            let bbox = BoundingBox { hplanes };
            let r = Grp3::from_columns(&[n1, n2, n3]);
            let squared_a_inv = 1.0 / a.powi(2);
            let s = Grp3::from_diagonal(&Vec3::new(squared_a_inv, squared_a_inv, squared_a_inv));
            let e = &r * &s * &r.transpose();
            let ellipsoid = Ellipsoid {
                center,
                r,
                a,
                b,
                c,
                e,
            };
            let line_ends = [p1.clone(), p2.clone()];
            Self {
                r_s,
                line_ends,
                bbox,
                ellipsoid,
            }
        }
    }

    #[derive(Debug)]
    pub struct Ellipsoid {
        /// center of the ellipsoid
        pub center: Vec3,
        /// rotation matrix of the ellipsoid
        pub r: Grp3,
        /// semi-axis a, b, c
        pub a: f64,
        pub b: f64,
        pub c: f64,
        /// matrix e = r*S*r'
        pub e: Grp3,
    }

    /// BoundingBox is defined by 6 planes.
    ///
    /// each plane is presented using the point-normal form: nx(x −x0) + ny(y − y0) + nz(z − z0) = 0.
    ///
    /// normal `n`: [nx,ny,nz]' , point `p`: [x0,y0,z0]'
    ///
    /// matrix form: n' (x - p) = 0, half-plane on the center's side: n' (x - p) < 0
    #[derive(Debug)]
    pub struct BoundingBox {
        pub hplanes: [HyperPlane; 6],
    }
    impl BoundingBox {
        pub fn is_point_inside_bbox(&self, x: &Vec3) -> bool {
            for plane in self.hplanes.iter() {
                if (x - &plane.p).dot(&plane.n) >= 0.0 {
                    return false;
                }
            }
            true
        }
    }
    #[derive(Debug)]
    pub struct HyperPlane {
        pub n: Vec3,
        pub p: Vec3,
    }
    impl HyperPlane {
        pub fn new(n: Vec3, center: &Vec3, d: &f64) -> Self {
            Self {
                n: n,
                p: center + &n * *d,
            }
        }
    }

    impl Ellipsoid {
        pub fn distance(&self, p: &Vec3) -> f64 {
            let dp = p - &self.center;
            let d = dp.transpose() * &self.e * &dp;
            d[0]
        }
    }

    /// x-axis is parallel to the line
    pub fn find_bbox_dir_x(l: &Vec3) -> Vec3 {
        let mut n1 = l.clone();
        n1 = n1.normalize();

        n1
    }

    ///  y-axis is perpendicular to the line and world z-axis
    pub fn find_bbox_dir_y(n1: &Vec3) -> Vec3 {
        let wd_z = Vec3::new(0.0, 0.0, 1.0);

        // if n1 is parallel to world z-axis, choose world y-axis as the y-axis
        if n1.dot(&wd_z) >= 0.999 {
            return Vec3::new(0.0, 1.0, 0.0);
        } else {
            return wd_z.cross(n1);
        }
    }

    /// z-axis is perpendiculat to the line and y-axis
    pub fn find_bbox_dir_z(n1: &Vec3, n2: &Vec3) -> Vec3 {
        n1.cross(&n2)
    }

    pub fn find_perpendicular_direction(n1: &Vec3) -> Vec3 {
        // let x = n1;
        // let x = n1.abs();
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
