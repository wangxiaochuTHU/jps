#[allow(unused)]
pub mod tests {
    use crate::base::JPS3DNeib;
    use crate::graphsearch::GraphSearch;
    use std::collections::HashSet;

    // #[test]
    pub fn test_jps3dneib() {
        let j = JPS3DNeib::default();
        println!("j = {:?}", j);
    }
    #[test]
    pub fn test_graph_search() {
        let map = [
            //↓ start
            0, 1, 1, 1, 1, // -> y = 0
            0, 1, 1, 1, 1, // -> y = 1
            0, 1, 1, 1, 1, // -> y = 2
            0, 1, 1, 1, 1, // -> y = 3
            0, 1, 1, 1, 1, // -> y = 4
            // z = 1
            1, 1, 1, 1, 1, //
            1, 1, 1, 1, 1, //
            1, 1, 1, 1, 1, //
            1, 1, 1, 1, 1, //
            0, 0, 0, 0, 0, //
            // z = 2
            1, 1, 1, 1, 0, //
            1, 1, 1, 1, 0, //
            1, 1, 1, 1, 0, //
            1, 1, 1, 1, 0, //
            1, 1, 1, 1, 0, //
            // z = 3
            0, 0, 0, 0, 0, //
            1, 1, 1, 1, 1, //
            1, 1, 1, 1, 1, //
            1, 1, 1, 1, 1, //
            1, 1, 1, 1, 1, //
            // z = 4
            0, 1, 1, 1, 1, //
            0, 0, 0, 1, 1, //
            0, 1, 0, 1, 1, //
            0, 1, 0, 0, 0, //
            0, 1, 1, 1, 0, // ← goal
        ];

        let mut occ_set: HashSet<(i32, i32, i32)> = HashSet::new();
        let mut free_set: HashSet<(i32, i32, i32)> = HashSet::new();

        for (b, idx) in map.into_iter().enumerate() {
            let z = b as i32 / 25;
            let y = (b as i32 - 25 * z) / 5;
            let x = b as i32 - 25 * z - 5 * y;
            if idx == 1 {
                // occupied coordinates
                occ_set.insert((x, y, z));
            } else {
                // free coordinates
                free_set.insert((x, y, z));
            }
        }
        let map_start = (0, 0, 0);
        let map_goal = (4, 4, 4);

        // or using,  GraphSearch::new_v1(Some(free_set), occ_set, 5, 5, 5, 1.0)
        let mut graphsearch = GraphSearch::new_v1(None, occ_set, [0, 5], [0, 5], [0, 5], 1.0);

        if graphsearch.plan_main(map_start, map_goal, true, 1000) {
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
            println!("");
            println!("turnings are :");
            for t in turnings.iter() {
                print!("{:?} ", t);
            }
        } else {
            println!("path finding failed");
        }
    }
}
