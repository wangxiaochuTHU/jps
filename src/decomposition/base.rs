#[cfg(feature = "decomp")]
#[allow(non_upper_case_globals)]
#[allow(unused)]
pub mod decomp_tool {
    use std::hash::Hash;

    use im::HashSet;
    use itertools::Itertools;
    pub use yakf::kf::{self, so3::SO3, Grp3, Vec3};
    // pub use yakf::linalg::*;
    pub const a_max: f64 = 10.0;
    pub const v_max: f64 = 5.0;
    pub const r_robot: f64 = 0.2;

    pub trait Voxelable {
        fn coordinate_to_grid(&self, pos: &Vec3) -> (i32, i32, i32) {
            unimplemented!();
        }
        fn grid_to_coordinate(&self, grid: &(i32, i32, i32)) -> Vec3 {
            unimplemented!();
        }
        fn get_grid_size(&self) -> (f64, f64, f64) {
            unimplemented!();
        }
    }
    #[derive(Debug)]
    pub struct Decomp<T: Voxelable + Hash + Sized + Send + Clone + Copy + Default> {
        pub trans: T,
        /// radius of safe flight,  r_s = v_max^2/(2*a_max)
        pub r_s: f64,

        /// line is defined by two points
        pub line_ends: [Vec3; 2],

        /// boundingbox
        pub bbox: BoundingBox,

        /// ellipsoid
        pub ellipsoid: Ellipsoid,
    }
    impl<T> Decomp<T>
    where
        T: Voxelable + Hash + Sized + Send + Clone + Copy + Default,
    {
        pub fn init(p1: &(i32, i32, i32), p2: &(i32, i32, i32), trans: T) -> Self {
            let p1 = trans.grid_to_coordinate(p1);
            let p2 = trans.grid_to_coordinate(p2);
            let r_s = 0.5 * v_max.powi(2) / a_max;
            let d = r_s + r_robot;
            let center = 0.5 * (&p1 + &p2);
            let line = &p2 - &p1;
            let a = line.norm();
            let b = a;
            let c = a;
            let n1 = find_bbox_dir_x(&line);
            let n2 = find_bbox_dir_y(&n1);
            let n3 = find_bbox_dir_z(&n1, &n2);
            let hplanes: [HyperPlane; 6] = [
                HyperPlane::new(n1, &center + &n1 * (0.5 * a + d)),
                HyperPlane::new(-n1, &center - &n1 * (0.5 * a + d)),
                HyperPlane::new(n2, &center + &n2 * d),
                HyperPlane::new(-n2, &center - &n2 * d),
                HyperPlane::new(n3, &center + &n3 * d),
                HyperPlane::new(-n3, &center - &n3 * d),
            ];
            let vertices: [Vec3; 8] = [
                &center + &n1 * (0.5 * a + d) + &n2 * d + &n3 * d,
                &center + &n1 * (0.5 * a + d) + &n2 * d - &n3 * d,
                &center + &n1 * (0.5 * a + d) - &n2 * d + &n3 * d,
                &center + &n1 * (0.5 * a + d) - &n2 * d - &n3 * d,
                &center - &n1 * (0.5 * a + d) + &n2 * d + &n3 * d,
                &center - &n1 * (0.5 * a + d) + &n2 * d - &n3 * d,
                &center - &n1 * (0.5 * a + d) - &n2 * d + &n3 * d,
                &center - &n1 * (0.5 * a + d) - &n2 * d - &n3 * d,
            ];
            let bbox = BoundingBox { hplanes, vertices };
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
            let line_ends = [p1, p2];
            Self {
                trans,
                r_s,
                line_ends,
                bbox,
                ellipsoid,
            }
        }

        pub fn get_grids_to_check(
            &self,
            omap: &HashSet<(i32, i32, i32)>,
        ) -> HashSet<(i32, i32, i32)> {
            let grids = self
                .bbox
                .vertices
                .iter()
                .map(|vertex| self.trans.coordinate_to_grid(vertex));
            let (mut i_min, mut i_max, mut j_min, mut j_max, mut k_min, mut k_max) = (
                std::i32::MAX,
                std::i32::MIN,
                std::i32::MAX,
                std::i32::MIN,
                std::i32::MAX,
                std::i32::MIN,
            );
            for grid in grids {
                if grid.0 < i_min {
                    i_min = grid.0;
                }
                if grid.0 > i_max {
                    i_max = grid.0;
                }
                if grid.1 < j_min {
                    j_min = grid.1;
                }
                if grid.1 > j_max {
                    j_max = grid.1;
                }
                if grid.2 < k_min {
                    k_min = grid.2;
                }
                if grid.2 > k_max {
                    k_max = grid.2;
                }
            }
            let mut check_set: HashSet<(i32, i32, i32)> = HashSet::new();
            for i in i_min..i_max + 1 {
                for j in j_min..j_min + 1 {
                    for k in k_min..k_max + 1 {
                        let grid = (i, j, k);
                        if omap.contains(&grid) {
                            let _ = check_set.insert(grid);
                        }
                    }
                }
            }
            check_set
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
        pub vertices: [Vec3; 8],
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
        pub fn new(n: Vec3, p: Vec3) -> Self {
            Self { n, p }
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
