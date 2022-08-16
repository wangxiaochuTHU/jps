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
}
