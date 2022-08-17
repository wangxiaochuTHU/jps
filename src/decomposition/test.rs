#[cfg(feature = "decomp")]
#[allow(unused)]
mod tests {
    use crate::decomposition::base::decomp_tool::{
        self, find_perpendicular_direction, find_third_direction, get_attitude_from_one_vector,
        Grp3, Vec3,
    };
    #[test]
    fn test_find_n2() {
        let mut n1 = Vec3::new(0.0, 5.0, -3.0);
        n1 = n1.normalize();
        let n2 = find_perpendicular_direction(&n1);
        println!("n2 = {:?}", n2.as_slice());

        assert!((n1.dot(&n2)).abs() < 1e-7);
        assert!((n2.norm() - 1.0).abs() < 1e-7);
    }
    #[test]
    fn test_find_n2n3() {
        let mut n1 = Vec3::new(0.0, 5.0, -3.0);
        n1 = n1.normalize();
        let n2 = find_perpendicular_direction(&n1);
        let n3 = find_third_direction(&n1, &n2);
        println!("n2 = {:?}", n2.as_slice());
        println!("n3 = {:?}", n3.as_slice());
        let r = get_attitude_from_one_vector(&n1);

        assert!((n1.dot(&n3)).abs() < 1e-7);
        assert!((n3.norm() - 1.0).abs() < 1e-7);
        assert!(((r.transpose() * r - Grp3::identity()).norm() < 1e-7));
    }

    /// This test outputs some vectors for manual check
    #[cfg(feature = "decomp")]
    #[test]
    fn test_decomp_init() {
        use crate::decomp_tool::{Decomp, Voxelable};
        #[derive(Debug, Hash, Clone, Copy)]
        pub struct Transformer {}
        impl Voxelable for &Transformer {
            fn coordinate_to_grid(&self, pos: &Vec3) -> (i32, i32, i32) {
                (
                    (pos[0] / 0.55).round() as i32,
                    (pos[1] / 0.45).round() as i32,
                    (pos[2] / 0.6).round() as i32,
                )
            }
            fn grid_to_coordinate(&self, grid: &(i32, i32, i32)) -> Vec3 {
                Vec3::new(
                    grid.0 as f64 * 0.55,
                    grid.1 as f64 * 0.45,
                    grid.2 as f64 * 0.6,
                )
            }
            fn get_grid_size(&self) -> (f64, f64, f64) {
                (0.55, 0.45, 0.6)
            }
        }
        let trans = Transformer {};
        let grid_end_1 = (5, -2, 6);
        let grid_end_2 = (8, 1, 9);
        let mut decomp = Decomp::init(&grid_end_1, &grid_end_2, &trans);
        println!("decomp.bbox.vertices = {:?}", decomp.bbox.vertices);
        println!("decomp.bbox.hplanes.n = ");
        for h in decomp.bbox.hplanes.iter() {
            println!("{:?},", h.n.as_slice());
        }
        println!("decomp.bbox.hplanes.p = ");
        for h in decomp.bbox.hplanes.iter() {
            println!("{:?},", h.p.as_slice());
        }

        println!(
            "[Before shrinking] decomp.ellipsoid.b , c = {} , {}",
            decomp.ellipsoid.b, decomp.ellipsoid.c
        );
        let p1 = &decomp.ellipsoid.center + Vec3::new(0.2, 0.1, 0.3);
        decomp.ellipsoid.shrink_two_axes_by_point(&p1);
        println!(
            "[After shrinking] decomp.ellipsoid.b , c = {} , {}",
            decomp.ellipsoid.b, decomp.ellipsoid.c
        );
        println!(
            "[After shrinking] distance between p1-ellipsoid = {}",
            decomp.ellipsoid.distance(&p1),
        );
        println!(
            "[After shrinking] ellipsoid contains p1?  {}",
            decomp.ellipsoid.contains(&p1),
        );
    }
}
