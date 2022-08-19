#[cfg(feature = "decomp")]
#[allow(non_upper_case_globals)]
#[allow(unused)]
pub mod decomp_tool {
    use std::hash::Hash;

    use itertools::Itertools;
    use std::collections::HashSet;
    pub use yakf::kf::{self, so3::SO3, Grp3, Vec3};
    // pub use yakf::linalg::*;
    // pub const a_max: f64 = 10.0;
    // pub const v_max: f64 = 5.0;
    // pub const r_robot: f64 = 0.2;

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
    #[derive(Debug, Clone)]
    pub struct Decomp<T: Voxelable + Hash + Sized + Send + Clone + Copy> {
        /// an object that can provide methods to transform physical coordinates and voxelized grids.
        pub trans: T,
        /// radius of safe flight,  r_s = v_max^2/(2*a_max)
        pub r_s: f64,

        /// line is defined by two points
        pub line_ends: [Vec3; 2],

        /// boundingbox
        pub bbox: BoundingBox,

        /// ellipsoid
        pub ellipsoid: Ellipsoid,

        /// polyhedron, defined by a serials of HyperPlanes
        pub polyhedron: Vec<HyperPlane>,
    }
    impl<T> Decomp<T>
    where
        T: Voxelable + Hash + Sized + Send + Clone + Copy,
    {
        /// init a Decomp
        pub fn init(
            g1: &(i32, i32, i32),
            g2: &(i32, i32, i32),
            trans: T,
            r_s: f64,
            r_robot: f64,
        ) -> Self {
            let p1 = trans.grid_to_coordinate(g1);
            let p2 = trans.grid_to_coordinate(g2);
            let d = r_s + r_robot;
            let center = 0.5 * (&p1 + &p2);
            let line = &p2 - &p1;
            let a = line.norm() / 2.0;
            let b = a;
            let c = a;
            let n1 = find_bbox_dir_x(&line);
            let n2 = find_bbox_dir_y(&n1);
            let n3 = find_bbox_dir_z(&n1, &n2);
            let hplanes: [HyperPlane; 6] = [
                HyperPlane::new(n1, &center + &n1 * (a + d)),
                HyperPlane::new(-n1, &center - &n1 * (a + d)),
                HyperPlane::new(n2, &center + &n2 * d),
                HyperPlane::new(-n2, &center - &n2 * d),
                HyperPlane::new(n3, &center + &n3 * d),
                HyperPlane::new(-n3, &center - &n3 * d),
            ];
            let vertices: [Vec3; 8] = [
                &center + &n1 * (a + d) + &n2 * d + &n3 * d,
                &center + &n1 * (a + d) + &n2 * d - &n3 * d,
                &center + &n1 * (a + d) - &n2 * d + &n3 * d,
                &center + &n1 * (a + d) - &n2 * d - &n3 * d,
                &center - &n1 * (a + d) + &n2 * d + &n3 * d,
                &center - &n1 * (a + d) + &n2 * d - &n3 * d,
                &center - &n1 * (a + d) - &n2 * d + &n3 * d,
                &center - &n1 * (a + d) - &n2 * d - &n3 * d,
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

            let polyhedron: Vec<HyperPlane> = Vec::new();
            Self {
                trans,
                r_s,
                line_ends,
                bbox,
                ellipsoid,
                polyhedron,
            }
        }

        /// find which grids need to check
        pub fn find_grids_to_check(
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

        /// inflate obstacles by a distance.
        /// Currently, the method is to move the obstable towards the center of the ellipsoid by a distance.
        /// But I feel this would not be exact for a part of the points.
        ///
        /// TODO: how to exactly and efficiently inflate? I think inflation should be done before path finding.
        pub fn inflate_obstacles(&self, opoints: &mut Vec<Vec3>, inflate_distance: f64) {
            for p in opoints.iter_mut() {
                let dir_cp = *p - self.ellipsoid.center;
                let dir_cp = dir_cp.normalize();
                *p = *p - dir_cp * inflate_distance;
            }
        }

        /// make the ellipsoid keep deforming to fit for surrounding occupied points, until finished
        pub fn best_fit_ellipsoid_for_occupied_points(
            &mut self,
            opoints: &Vec<Vec3>,
        ) -> (Option<usize>, Option<usize>) {
            let (mut k1, mut k2): (Option<usize>, Option<usize>) = (None, None); // two vars to restore index of the found points.

            /* step 1: shrink two axes. */
            loop {
                // calculate all distances from the ellipsoid
                let darray = opoints
                    .iter()
                    .enumerate()
                    .map(|(i, p)| (i, self.ellipsoid.distance(p)));
                // find the index satisfying: 1) distance less than 0.9999 && 2) with the minimal distance
                let idx = darray
                    .filter(|(i, d)| *d < 0.9999)
                    .min_by(|id1, id2| id1.1.partial_cmp(&id2.1).unwrap());
                // if found none, finished loop; else, shrink for passing that point and then continue the loop
                match idx {
                    None => break,
                    Some((k, d)) => {
                        self.ellipsoid.shrink_two_axes_by_point(&opoints[k]); // shrink
                        k1 = Some(k); // update `k1`
                                      // println!("d = {}", d);
                    }
                }
            }
            /* step 2: reset the two axes of the ellipsoid*/
            match k1 {
                // None means there is no intersected point. in this case, the ellipsoid needs no more adjustments.
                None => return (k1, k2),
                // if intersected point exists, reset and find a next intersected point.
                Some(kk1) => {
                    // find direction n3
                    let op = &opoints[kk1] - &self.ellipsoid.center;
                    let op = op.normalize();
                    let n1 = &self.ellipsoid.r.index((0..3, 0));
                    let n3 = n1.cross(&op);

                    // // test reset
                    // println!(
                    //     "----------------- d1 = {}, d2 = {}",
                    //     self.ellipsoid.distance(&self.line_ends[0]),
                    //     self.ellipsoid.distance(&self.line_ends[1])
                    // );

                    // reset
                    self.ellipsoid.reset_two_axes(n3, &opoints[kk1]);

                    // // test reset
                    // println!(
                    //     "++++++++++++++++++ d1 = {}, d2 = {}",
                    //     self.ellipsoid.distance(&self.line_ends[0]),
                    //     self.ellipsoid.distance(&self.line_ends[1])
                    // );
                }
            }

            /* step 3: shrink z-axis */
            loop {
                // calculate all distances from the ellipsoid
                let darray = opoints
                    .iter()
                    .enumerate()
                    .map(|(i, p)| (i, self.ellipsoid.distance(p)));
                // find the index satisfying: 1) distance less than 0.9999 && 2) with the minimal distance
                let idx = darray
                    .filter(|(i, d)| *d < 0.999)
                    .min_by(|id1, id2| id1.1.partial_cmp(&id2.1).unwrap());
                // if found none, finished loop; else, shrink for passing that point and then continue the loop
                match idx {
                    None => break,
                    Some((k, d)) => {
                        self.ellipsoid.shrink_one_axes_by_point(&opoints[k]); // shrink
                        k2 = Some(k); // update `k2`
                    }
                }
            }
            return (k1, k2);
        }

        /// check if Point p is on the same side
        #[inline]
        pub fn is_at_inner_side(p: &Vec3, polys: &Vec<HyperPlane>) -> bool {
            for hp in polys.iter() {
                if hp.n.dot(&(p - &hp.p)) >= -1e-4 {
                    return false;
                }
            }
            true
        }

        /// cut the space surrounding the ellipsoid
        pub fn cut_into_polyhedron(
            &mut self,
            mut opoints: Vec<Vec3>,
            k1: Option<usize>,
            k2: Option<usize>,
            r_robot: f64,
        ) {
            let block = &self.ellipsoid.e + &self.ellipsoid.e.transpose();
            // add hyperplane by k1
            if let Some(k) = k1 {
                let p = opoints[k];
                let n = &block * (&p - &self.ellipsoid.center);
                let n = n.normalize();
                self.polyhedron.push(HyperPlane { n, p });
            }
            // add hyperplane by k2
            if let Some(k) = k2 {
                let p = opoints[k];
                let n = &block * (&p - &self.ellipsoid.center);
                let n = n.normalize();
                self.polyhedron.push(HyperPlane { n, p });
            }

            // remove from `opoints` those points that are on the oppsite side w.r.t k1 and k2 hyperplanes
            opoints.retain(|p| Self::is_at_inner_side(p, &self.polyhedron));
            loop {
                // calculate all distances from the ellipsoid
                let darray = opoints
                    .iter()
                    .enumerate()
                    .map(|(i, p)| (i, self.ellipsoid.distance(p)));
                // find the index satisfying: 1) distance more than 1.0001 && 2) with the minimal distance
                let idx = darray
                    .filter(|(i, d)| *d > 1.0001)
                    .min_by(|id1, id2| id1.1.partial_cmp(&id2.1).unwrap());
                // if found none, finished loop; else, dilate for passing that point and then continue the loop
                match idx {
                    None => break,
                    Some((k, d)) => {
                        self.ellipsoid.dilate_by_point(&opoints[k]); // dilate

                        // add the hyperplane
                        let block = &self.ellipsoid.e + &self.ellipsoid.e.transpose();
                        let p = opoints[k];
                        let n = &block * (&p - &self.ellipsoid.center);
                        let n = n.normalize();
                        self.polyhedron.push(HyperPlane { n, p });

                        // remove from `opoints` those points that are on the oppsite side
                        opoints.retain(|p| Self::is_at_inner_side(p, &self.polyhedron));
                    }
                }
            }
            // finally, put the 6 hyperplanes that the bbox holds into polyhedron.
            self.polyhedron.extend(self.bbox.hplanes.into_iter());

            // // adjust hyperplane to ensure minimal distance to the Line >= r_robot
            // let line_dir = &self.line_ends[1] - &self.line_ends[0];
            // let line_dir = line_dir.normalize();
            // for hp in self.polyhedron.iter_mut() {
            //     let d1 = ((&self.line_ends[0] - &hp.p).dot(&hp.n)).abs();
            //     let d2 = ((&self.line_ends[1] - &hp.p).dot(&hp.n)).abs();
            //     let cos_angle = (&line_dir).dot(&hp.n);
            //     if (d1 < r_robot || d2 < r_robot) && cos_angle.abs() > 1e-4 {
            //         let t = &hp.n - cos_angle * line_dir;
            //         let mut t = t.normalize();
            //         if t.dot(&hp.n).is_sign_negative() {
            //             t = -t;
            //         }
            //         hp.n = t;
            //     }
            // }
        }
    }

    #[derive(Debug, Clone, Copy)]
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
    impl Ellipsoid {
        /// shrink two axes (b,c) simultaneously, such that Point p is on the surface of the ellipsoid.
        pub fn shrink_two_axes_by_point(&mut self, p: &Vec3) {
            let s0 = 1.0 / self.a.powi(2);
            let y = self.r.transpose() * (p - &self.center);
            let s_1_2 = (1.0 - y[0].powi(2) * s0) / (y[1].powi(2) + y[2].powi(2));
            let new_b = 1.0 / s_1_2.sqrt();
            self.b = new_b;
            self.c = new_b;

            self.e =
                &self.r * Grp3::from_diagonal(&Vec3::new(s0, s_1_2, s_1_2)) * &self.r.transpose();
        }

        /// assign a new axis direction for the z-axis, and reset the z-axis equal to `a`.
        pub fn reset_two_axes(&mut self, n3: Vec3, p: &Vec3) {
            // println!("-------r = {:.4?}", self.r);
            // let y = self.r.transpose() * (p - &self.center);
            let n1 = &self.r.index((0..3, 0));
            let n2 = n3.cross(n1);
            self.r.index_mut((0..3, 1)).copy_from(&n2);
            self.r.index_mut((0..3, 2)).copy_from(&n3);

            let z = self.r.transpose() * (p - &self.center);

            let s_1_2 = (1.0 - z[0].powi(2) / self.a.powi(2)) / (z[1].powi(2) + z[2].powi(2));
            let new_b = 1.0 / s_1_2.sqrt();
            self.b = new_b;
            // self.e =
            //     &self.r * Grp3::from_diagonal(&Vec3::new(s0, s_1_2, s_1_2)) * &self.r.transpose();
            //reset
            self.c = self.a;
            self.e = &self.r
                * Grp3::from_diagonal(&Vec3::new(
                    1.0 / self.a.powi(2),
                    1.0 / self.b.powi(2),
                    1.0 / self.c.powi(2),
                ))
                * &self.r.transpose();

            // // println!("+++++++r = {:.4?}", self.r);
            // self.b = self.a;
            // self.c = self.b;

            // // // self.c = self.a;
            // self.e = &self.r
            //     * Grp3::from_diagonal(&Vec3::new(
            //         1.0 / self.a.powi(2),
            //         1.0 / self.b.powi(2),
            //         1.0 / self.c.powi(2),
            //     ))
            //     * &self.r.transpose();
            // self.shrink_two_axes_by_point(p);
            // // reset c
            // self.c = self.a;
            // self.e = &self.r
            //     * Grp3::from_diagonal(&Vec3::new(
            //         1.0 / self.a.powi(2),
            //         1.0 / self.b.powi(2),
            //         1.0 / self.c.powi(2),
            //     ))
            //     * &self.r.transpose();
        }

        /// shrink z-axis (c), such that Point p is on the surface of the ellipsoid.
        pub fn shrink_one_axes_by_point(&mut self, p: &Vec3) {
            let s0 = 1.0 / self.a.powi(2);
            let s1 = 1.0 / self.b.powi(2);
            let y = self.r.transpose() * (p - &self.center);
            // considering a special case, where y[2] = 0
            let s2 = (1.0 - y[0].powi(2) * s0 - y[1].powi(2) * s1) / y[2].powi(2);
            let new_c = 1.0 / s2.sqrt();
            self.c = new_c;
            // println!("second=================");
            // if new_c < 0.001 {
            //     println!("");
            //     println!(
            //         "center = {:.4?}, p = {:.4?}, p-c = {:.4?}, d = {:.4?}, y2 = {:.?}, ellip = {:.4?}",
            //         self.center,
            //         p,
            //         p - self.center,
            //         self.distance(p),
            //         y[2],
            //         self
            //     );
            //     // println!("");
            // }

            self.e = &self.r * Grp3::from_diagonal(&Vec3::new(s0, s1, s2)) * &self.r.transpose();
            // println!("new d = {:.4?}", self.distance(p));
            // println!("");
        }

        /// dilate all three axes (a,b,c), such that Point p is on the surface of the ellipsoid.
        pub fn dilate_by_point(&mut self, p: &Vec3) {
            let s0 = 1.0 / self.a.powi(2);
            let s1 = 1.0 / self.b.powi(2);
            let s2 = 1.0 / self.c.powi(2);
            let y = self.r.transpose() * (p - &self.center);
            let β = y[0].powi(2) * s0 + y[1].powi(2) * s1 + y[2].powi(2) * s2;

            let s = Grp3::from_diagonal(&Vec3::new(s0 / β, s1 / β, s2 / β));
            self.e = &self.r * s * &self.r.transpose();
            self.a *= β.sqrt();
            self.b *= β.sqrt();
            self.c *= β.sqrt();
        }

        /// check whether Point p is inside the ellipsoid
        pub fn contains(&self, p: &Vec3) -> bool {
            if self.distance(p) < 0.9999 {
                true
            } else {
                false
            }
        }
        #[inline]
        pub fn distance(&self, p: &Vec3) -> f64 {
            let dp = p - &self.center;
            let d = dp.transpose() * &self.e * &dp;
            d[0].abs()
        }
    }

    /// BoundingBox is defined by 6 planes.
    ///
    /// each plane is presented using the point-normal form: nx(x −x0) + ny(y − y0) + nz(z − z0) = 0.
    ///
    /// normal `n`: [nx,ny,nz]' , point `p`: [x0,y0,z0]'
    ///
    /// matrix form is: n' (x - p) = 0, half-plane on the center's side meets: n' (x - p) < 0
    #[derive(Debug, Clone, Copy)]
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
    #[derive(Debug, Clone, Copy)]
    pub struct HyperPlane {
        pub n: Vec3,
        pub p: Vec3,
    }
    impl HyperPlane {
        pub fn new(n: Vec3, p: Vec3) -> Self {
            Self { n, p }
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
