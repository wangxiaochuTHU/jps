#[cfg(feature = "decomp")]
#[allow(unused)]
mod tests {

    use yakf::{kf::One2OneMapSO3, lie::so3::SO3, linalg::Const};
    pub fn make_obstacles_in_cuboid(
        center: Vec3,
        a: f64,
        b: f64,
        c: f64,
        interval: f64,
        attitude: Grp3,
    ) -> Vec<Vec3> {
        let mut opoints: Vec<Vec3> = Vec::new();
        let step_x = (a / interval).floor() as i32;
        let step_y = (b / interval).floor() as i32;
        let step_z = (c / interval).floor() as i32;
        for i in -step_x..step_x + 1 as i32 {
            let x = -a + i as f64 * interval;
            for j in -step_y..step_y + 1 as i32 {
                let y = -b + j as f64 * interval;
                for k in -step_z..step_z + 1 as i32 {
                    let z = -c + k as f64 * interval;
                    // let p_b = Vec3::new(x, y, z);
                    // let p_w = attitude.transpose() * p_b + center;
                    // opoints.push(p_w)
                    let tmp = i.abs() + j.abs() + k.abs();
                    if tmp >= step_x || tmp >= step_y || tmp >= step_z {
                        let p_b = Vec3::new(x, y, z);
                        let p_w = attitude.transpose() * p_b + center;
                        opoints.push(p_w)
                    }
                }
            }
        }
        opoints
    }

    pub fn make_obstacles_in_cylinder(
        center: Vec3,
        r: f64,
        h: f64,
        interval: f64,
        attitude: Grp3,
    ) -> Vec<Vec3> {
        let mut opoints: Vec<Vec3> = Vec::new();

        let steps_r = (2.0 * PI * r / interval * 1.05).floor() as i32;
        let da = 2.0 * PI / steps_r as f64;
        let steps_h = (h / interval).floor() as i32;
        for i in 0..steps_r {
            let x = (da * i as f64).cos() * r;
            let y = (da * i as f64).sin() * r;
            for j in -steps_h..steps_h {
                let z = j as f64 * interval;

                let p_b = Vec3::new(x, y, z);
                let p_w = attitude.transpose() * p_b + center;
                opoints.push(p_w)
            }
        }
        opoints
    }

    #[test]
    pub fn test_map() {
        use crate::base::JPS3DNeib;
        use crate::graphsearch::GraphSearch;
        use std::collections::{HashMap, HashSet};
        let opoints = generate_simple_world_map(24.0, 24.0, 6.0);
        println!("opoints contains {} points", opoints.len());
        use std::fs::File;
        use std::io::Write;
        let mut w = File::create("opoints.txt").unwrap();
        for p in opoints.iter() {
            let s = format!("{},{},{}", p[0], p[1], p[2]);
            writeln!(&mut w, "{}", s).unwrap();
        }

        // Voxelization
        use crate::decomp_tool::{Decomp, Voxelable};
        use rand::prelude::*;
        #[derive(Debug, Hash, Clone, Copy)]
        pub struct Transformer {}
        impl Voxelable for Transformer {
            fn coordinate_to_grid(&self, pos: &Vec3) -> (i32, i32, i32) {
                (
                    (pos[0] / 0.45).round() as i32,
                    (pos[1] / 0.45).round() as i32,
                    (pos[2] / 0.45).round() as i32,
                )
            }
            fn grid_to_coordinate(&self, grid: &(i32, i32, i32)) -> Vec3 {
                Vec3::new(
                    grid.0 as f64 * 0.45,
                    grid.1 as f64 * 0.45,
                    grid.2 as f64 * 0.45,
                )
            }
            fn get_grid_size(&self) -> (f64, f64, f64) {
                (0.45, 0.45, 0.45)
            }
        }
        let a_max: f64 = 5.0;
        let v_max: f64 = 3.0;
        let r_robot: f64 = 0.2;
        let r_s = 0.5 * v_max.powi(2) / a_max;
        let trans = Transformer {};

        let mut occ_container_map: HashMap<(i32, i32, i32), Vec<&Vec3>> = HashMap::new();
        let mut occ_set: HashSet<(i32, i32, i32)> = HashSet::new();
        for p in opoints.iter() {
            let grid = trans.coordinate_to_grid(p);
            if occ_container_map.contains_key(&grid) {
                let v = occ_container_map.get_mut(&grid).unwrap();
                v.push(p);
            } else {
                occ_container_map.insert(grid, vec![p]);
            }
            let _ = occ_set.insert(grid);
        }

        // set the start and goal
        let mut grid_start = (0, 0, 0);
        let mut grid_goal = (0, 0, 0);
        'loop1: for x in -8..5 {
            for y in -5..5 {
                for z in -3..3 {
                    if !occ_set.contains(&(x, y, z)) {
                        grid_start = (x, y, z);
                        break 'loop1;
                    }
                }
            }
        }

        'loop2: for x in 50..60 {
            for y in 52..60 {
                for z in 4..8 {
                    if !occ_set.contains(&(x, y, z)) {
                        grid_goal = (x, y, z);
                        break 'loop2;
                    }
                }
            }
        }

        let mut start = trans.grid_to_coordinate(&grid_start);
        let mut goal = trans.grid_to_coordinate(&grid_goal);

        // use JPS the find line segments
        let mut graphsearch =
            GraphSearch::new_v1(None, occ_set, [-10, 80], [-10, 80], [-10, 80], 1.0);

        let t1 = std::time::Instant::now();
        if graphsearch.plan_main(grid_start, grid_goal, true, 2000) {
            let t2 = std::time::Instant::now();
            println!("path finding cost {} us", (t2 - t1).as_secs_f64() * 1e6);
            let path = &graphsearch.path_;
            let turnings = &graphsearch.turnings;
            let fmt_path = path.iter().enumerate().map(|(i, x)| {
                if i > 0 {
                    format!("➡ {:?}", x.id)
                } else {
                    format!("{:?} ", x.id)
                }
            });
            println!("path is :");
            for node in fmt_path {
                print!("{}", node);
            }
            let mut w = File::create("turnings.txt").unwrap();
            println!("");
            println!("turnings are :");
            for t in turnings.iter() {
                print!("{:?} ", t);
                let t_coord = trans.grid_to_coordinate(t);
                let s = format!("{},{},{}", t_coord[0], t_coord[1], t_coord[2]);
                writeln!(&mut w, "{}", s).unwrap();
            }
            println!("");

            // decomposition
            let mut decomps: Vec<Decomp<Transformer>> = Vec::new();

            let t1 = std::time::Instant::now();
            for k in 1..turnings.len() {
                let grid_end_1 = &turnings[k - 1];
                let grid_end_2 = &turnings[k];
                let mut decomp = Decomp::init(grid_end_1, grid_end_2, trans, r_s, r_robot);
                let grids_to_check = decomp.find_grids_to_check(&graphsearch.omap_);
                let mut local_opoints: Vec<Vec3> = Vec::new();
                for g in grids_to_check.iter() {
                    let v = occ_container_map.get(g).unwrap();
                    for p in v.iter() {
                        local_opoints.push(*p.clone());
                    }
                }
                decomp.inflate_obstacles(&mut local_opoints, r_robot); // inflate_obstacles

                let (k1, k2) = decomp.best_fit_ellipsoid_for_occupied_points(&local_opoints);
                decomp.cut_into_polyhedron(local_opoints, k1, k2, r_robot);
                decomps.push(decomp);
            }
            let t2 = std::time::Instant::now();
            println!(
                "finding ellipsoids and cutting, cost time {} us",
                (t2 - t1).as_secs_f64() * 1e6
            );

            // write all ellipsoid into txt for matlab check
            let mut w = File::create("ellipsoids.txt").unwrap();
            for dec in decomps.iter() {
                let ellip_center = dec.ellipsoid.center;
                let ellip_e = dec.ellipsoid.e.reshape_generic(Const::<9>, Const::<1>);
                let mut s = String::new();
                for i in 0..3 {
                    s += format!("{},", ellip_center[i]).as_str();
                }
                for i in 0..9 {
                    s += format!("{},", ellip_e[i]).as_str();
                }
                writeln!(&mut w, "{}", s).unwrap();
                println!(
                    "ellipsoid a,b,c = {:.3}, {:.3}, {:.3},  polyhedron has {} hyperplanes",
                    dec.ellipsoid.a,
                    dec.ellipsoid.b,
                    dec.ellipsoid.c,
                    dec.polyhedron.len(),
                );
            }
        } else {
            println!("path finding failed");
        }

        /// generate a world map with simple 3D shapes.
        pub fn generate_simple_world_map(length: f64, width: f64, height: f64) -> Vec<Vec3> {
            use rand::distributions::Uniform;
            let mut opoints: Vec<Vec3> = Vec::new();
            use rand::prelude::*;

            let mut x_rng = Uniform::new(0_f64, length);
            let mut y_rng = Uniform::new(0_f64, width);
            let mut z_rng = Uniform::new(0_f64, height);
            for i in 0..35 {
                let mut rng = thread_rng();
                let center = Vec3::new(
                    x_rng.sample(&mut rng),
                    y_rng.sample(&mut rng),
                    z_rng.sample(&mut rng),
                );
                let v = Vec3::new(x_rng.sample(&mut rng), rng.gen(), z_rng.sample(&mut rng));
                let attitude = SO3::from_vec(v).to_grp();
                match rng.gen_bool(0.5) {
                    true => {
                        let r = rng.gen_range(length / 40.0..length / 30.0);
                        let h = rng.gen_range(height / 20.0..height / 10.0);
                        let mut ops = make_obstacles_in_cylinder(center, r, h, 0.15, attitude);
                        opoints.append(&mut ops)
                    }
                    false => {
                        let a = rng.gen_range(length / 40.0..length / 25.0);
                        let b = rng.gen_range(width / 20.0..width / 10.0);
                        let c = rng.gen_range(height / 20.0..height / 10.0);
                        let mut ops = make_obstacles_in_cuboid(center, a, b, c, 0.15, attitude);
                        opoints.append(&mut ops)
                    }
                }
            }
            opoints
        }
    }

    use std::f64::consts::PI;

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

    /// This test also outputs some vectors for manual check
    #[cfg(feature = "decomp")]
    #[test]
    fn test_decomp_init() {
        use crate::decomp_tool::{Decomp, Voxelable};
        use rand::prelude::*;
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
        let a_max: f64 = 10.0;
        let v_max: f64 = 5.0;
        let r_robot: f64 = 0.2;
        let r_s = 0.5 * v_max.powi(2) / a_max;
        let trans = Transformer {};
        let grid_end_1 = (5, -2, 6);
        let grid_end_2 = (8, 1, 9);
        let mut decomp = Decomp::init(&grid_end_1, &grid_end_2, &trans, r_s, r_robot);
        let mut decomp2 = decomp.clone();

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
            "[After shrinking] ellipsoid strictly contains p1?  {}",
            decomp.ellipsoid.contains(&p1),
        );
        assert!((decomp.ellipsoid.distance(&p1) - 1.0).abs() < 1e-5);
        assert_eq!(decomp.ellipsoid.contains(&p1), false);

        // generate 50 random occupied points around the center
        let mut rng = thread_rng();
        let mut opoints: Vec<Vec3> = (0..50)
            .into_iter()
            .map(|i| {
                &decomp2.bbox.hplanes[2 + i as usize % 4].p
                    + 0.1
                        * Vec3::new(
                            rng.gen_range(-0.5_f64..0.5f64),
                            rng.gen_range(-0.8_f64..0.8f64),
                            rng.gen_range(-0.5_f64..0.5f64),
                        )
            })
            .collect();

        // test fit the ellipsoid
        let distances: Vec<&Vec3> = opoints
            .iter()
            .filter(|p| decomp2.ellipsoid.distance(p) < 0.9999)
            .collect();

        println!(
            "[Before fitting] ellipsoid a ={}, b ={}, c={}, contains {} opoints",
            decomp2.ellipsoid.a,
            decomp2.ellipsoid.b,
            decomp2.ellipsoid.c,
            distances.len()
        );
        let t_start = std::time::Instant::now();
        decomp2.inflate_obstacles(&mut opoints, r_robot); // inflate_obstacles
        let (k1, k2) = decomp2.best_fit_ellipsoid_for_occupied_points(&opoints);
        let t_end = std::time::Instant::now();
        let distances: Vec<&Vec3> = opoints
            .iter()
            .filter(|p| decomp2.ellipsoid.distance(p) < 0.9999)
            .collect();
        println!("cost {} us to fit", (t_end - t_start).as_secs_f64() * 1e6);
        println!(
            "[After fitting] ellipsoid a ={}, b ={}, c={}, contains {} opoints",
            decomp2.ellipsoid.a,
            decomp2.ellipsoid.b,
            decomp2.ellipsoid.c,
            distances.len()
        );
        assert!(distances.len() == 0);

        // cut_into_polyhedron
        println!(
            "[Before cutting] polyhedron has {} hyperplanes",
            decomp2.polyhedron.len(),
        );
        decomp2.cut_into_polyhedron(opoints, k1, k2, r_robot);
        println!(
            "[After cutting] polyhedron has {} hyperplanes",
            decomp2.polyhedron.len(),
        );
        // the center of the ellipsoid should be inside the polyhedron
        assert_eq!(
            Decomp::<&Transformer>::is_at_inner_side(
                &decomp2.ellipsoid.center,
                &decomp2.polyhedron
            ),
            true
        );
    }

    // fn test_map
}
