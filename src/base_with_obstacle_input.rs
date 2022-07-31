use crate::linalg::{OVector, U3};
use itertools::Itertools;
use std::collections::HashSet;

pub type Grid = OVector<i32, U3>;
pub type Dir = OVector<i32, U3>;
use super::constant::ALL_DIRS;

#[derive(Debug, Default, Clone)]
pub struct Node {
    pub pos: Grid,
    pub id: usize,
    pub fa: usize,
    pub dir: Vec<Dir>,
}

/// main entry of the jump point searching algorithm
/// `occ_set`: suppose all occupied grids have been collected into a HashSet
/// `x_max` means the maximum on x-axis.
/// `map_start` is the starting grid on the grid map
/// `map_goal` is the goal on the grid map.

pub fn jps_3d_v2(
    occ_set: &HashSet<Grid>,
    map_start: &Grid,
    map_goal: &Grid,
    x_max: i32,
    y_max: i32,
    z_max: i32,
) -> Option<Vec<Grid>> {
    let nodestart = Node {
        pos: *map_start,        // index of the point on the grid-map
        id: 0,                  // id records the index of node in closelist
        fa: 0,                  // fa records the index of the parent node in the closelist
        dir: ALL_DIRS.to_vec(), // searching directions
    };
    let mut map_visit: HashSet<Grid> = HashSet::new(); // map that has been visited

    /* openlist has tuple elements, i.e.  (node, fcost)
    where fcost = gn(EuclideanDistance) + hn(ManhattanDistance) */
    let mut openlist = vec![(nodestart.clone(), 0.0)];
    let mut closelist = vec![nodestart];

    map_visit.insert(*map_start);

    let mut findgoal = false;

    while openlist.len() > 0 {
        openlist.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap()); //sort by value in descending order
        let (node, _fcost) = openlist.pop().unwrap(); // take the node with the smallest fcost
        for d in node.dir.iter() {
            if let Some(mut nodenew) = jump(occ_set, map_goal, &node.pos, d, x_max, y_max, z_max) {
                // when the new grid hasn't been visited, visit it.
                if !map_visit.contains(&nodenew.pos) {
                    nodenew.fa = node.id;
                    nodenew.id = closelist.len();
                    let fnew = manhattan_distance(map_goal, &nodenew.pos)
                        + euclidean_distance(&node.pos, &nodenew.pos);
                    openlist.push((nodenew.clone(), fnew));
                    closelist.push(nodenew.clone());
                    map_visit.insert(nodenew.pos);

                    // find the goal
                    if nodenew.pos == *map_goal {
                        findgoal = true;
                        break;
                    }
                }
            }
        }
        // find the goal
        if findgoal {
            break;
        }
    }

    if findgoal {
        let mut path: Vec<Grid> = Vec::new();
        let mut k = closelist.len() - 1;

        // construct the path backwards.
        while k > 0 {
            let renode = &closelist[k];
            path.push(renode.pos);
            k = renode.fa;
        }
        path.push(*map_start);
        // reverse the path
        path.reverse();

        Some(path)
    } else {
        None
    }
}

#[allow(unused)]
pub fn euclidean_distance(pos1: &Grid, pos2: &Grid) -> f32 {
    let d = pos1 - pos2;
    d.into_iter().map(|i| i.pow(2) as f32).sum::<f32>().sqrt()
}

#[allow(unused)]
pub fn manhattan_distance(pos1: &Grid, pos2: &Grid) -> f32 {
    let d = pos1 - pos2;
    d.into_iter().map(|i| i.abs() as f32).sum()
}

#[allow(unused)]
pub fn jump(
    occ_set: &HashSet<Grid>,
    goal: &Grid,
    startpos: &Grid,
    d: &Dir,
    x_max: i32,
    y_max: i32,
    z_max: i32,
) -> Option<Node> {
    // println!("startpos = {:?}", startpos);

    let norm_d: u32 = d.into_iter().map(|i| i.abs() as u32).sum();
    if norm_d == 0 {
        return None;
    }

    let newpos = step(startpos, d);

    // ourside the grid on the newpos
    if is_outmap_check(&newpos, x_max, y_max, z_max) {
        return None;
    }
    // meet the obstacle on the newpos
    else if is_ob_check(&newpos, occ_set) {
        return None;
    }
    // if it's the goal. in this case, dir doesn't matter.
    else if newpos == *goal {
        let newnode = Node {
            pos: newpos,
            ..Default::default()
        };
        return Some(newnode);
    }

    if norm_d == 2 {
        let dside = dir_2dto1d(d);
        if is_ob_check(&(startpos + dside[0]), occ_set)
            && is_ob_check(&(startpos + dside[1]), occ_set)
        {
            return None;
        }
    }

    let mut dirs = get_forceneighbour_dirs(&newpos, d, occ_set, x_max, y_max, z_max);
    if dirs.len() > 0 {
        let newnode = Node {
            pos: newpos,
            dir: dirs,
            ..Default::default()
        };
        return Some(newnode);
    }

    // go 3d diagonal
    if norm_d == 3 {
        // one 3d-diagonal change three 2d-diagonal + three straght line
        let do6 = [
            Dir::new(0, d[1], d[2]),
            Dir::new(d[0], 0, d[2]),
            Dir::new(d[0], d[1], 0),
            Dir::new(d[0], 0, 0),
            Dir::new(0, d[1], 0),
            Dir::new(0, 0, d[2]),
        ];

        let mut dir: Vec<Dir> = vec![*d];
        for dk in do6.into_iter() {
            if jump(occ_set, goal, &newpos, &dk, x_max, y_max, z_max).is_some() {
                // Add the direction of find ForceNegihbour
                dir.push(dk);
            }
        }

        // find a ForceNegihbour
        if dir.len() > 1 {
            let newnode = Node {
                pos: newpos,
                dir: dir,
                ..Default::default()
            };
            return Some(newnode);
        }
    }

    // go 2d diagonal
    if norm_d == 2 {
        let do3 = [
            Dir::new(d[0], 0, 0),
            Dir::new(0, d[1], 0),
            Dir::new(0, 0, d[2]),
        ];
        let mut dir: Vec<Dir> = vec![*d];
        for dk in do3.into_iter() {
            if jump(occ_set, goal, &newpos, &dk, x_max, y_max, z_max).is_some() {
                // Add the direction of find ForceNegihbour
                dir.push(dk);
            }
        }

        // find a ForceNegihbour
        if dir.len() > 1 {
            let newnode = Node {
                pos: newpos,
                dir: dir,
                ..Default::default()
            };
            return Some(newnode);
        }
    }

    // [go straght] or [go diagonal not find ForceNegihbour]
    let newnode = jump(occ_set, goal, &newpos, d, x_max, y_max, z_max);
    return newnode;
}

/// suppose occ_set is HashSet of all occupied grids.
/// false means free, true means occupied
/// #[inline]
pub fn is_ob_check(pos: &Grid, occ_set: &HashSet<Grid>) -> bool {
    occ_set.contains(pos)
}

pub fn dir_2dto1d(d: &Dir) -> [Dir; 2] {
    let mut dside = [Dir::zeros(), Dir::zeros()];
    let mut k = 0;
    if d[0] != 0 {
        dside[k][0] = d[0];
        k += 1;
    }
    if d[1] != 0 {
        dside[k][1] = d[1];
        k += 1;
    }
    if d[2] != 0 {
        dside[k][2] = d[2];
    }

    dside
}

/// suppose grid coordinates applies the design:  x has xdim grids (similar for y, z), and,
/// 0 <= x <= x_max, xdim = x_max + 1
/// `is_outmap_check` returning true means out of map
/// `is_outmap_check` returning false means inside map
#[inline]
pub fn is_outmap_check(pos: &Grid, x_max: i32, y_max: i32, z_max: i32) -> bool {
    if pos[0] < 0 || pos[0] > x_max || pos[1] < 0 || pos[1] > y_max || pos[2] < 0 || pos[2] > z_max
    {
        return true;
    }
    false
}

pub fn has_forceneighbour_check(
    pos: &Grid,
    d: &Dir,
    occ_set: &HashSet<Grid>,
    x_max: i32,
    y_max: i32,
    z_max: i32,
) -> bool {
    let pos_nb = pos + d;
    if is_outmap_check(pos, x_max, y_max, z_max)
        || !is_ob_check(pos, occ_set)
        || is_outmap_check(&pos_nb, x_max, y_max, z_max)
        || is_ob_check(&pos_nb, occ_set)
    {
        return false;
    }
    true
}

pub fn get_forceneighbour_dirs(
    pos: &Grid,
    d: &Dir,
    occ_set: &HashSet<Grid>,
    x_max: i32,
    y_max: i32,
    z_max: i32,
) -> Vec<Dir> {
    let mut dirs: Vec<Dir> = Vec::new();
    if is_outmap_check(&(pos + d), x_max, y_max, z_max) {
        return dirs;
    }

    let norm_d: i32 = d.into_iter().map(|i| i.abs() as i32).sum();

    // [go straght]  ----------------------------------------------
    if norm_d == 1 {
        if is_ob_check(&(pos + d), occ_set) {
            return dirs;
        }
        let d1 = Dir::new(d[2].abs(), d[0].abs(), d[1].abs());
        let d2 = Dir::new(d[1].abs(), d[2].abs(), d[0].abs());
        let dob = [d1, -d1, d2, -d2, d1 + d2, -d1 - d2, d1 - d2, -d1 + d2];
        for dk in dob.into_iter() {
            if has_forceneighbour_check(&(pos + dk), d, occ_set, x_max, y_max, z_max) {
                dirs.push(dk + d);
            }
        }
    }
    // [go 2D-diagonal] ----------------------------------------------
    else if norm_d == 2 {
        let dside = dir_2dto1d(d);
        for ds in dside.into_iter() {
            if has_forceneighbour_check(&(pos - ds), &(d - ds), occ_set, x_max, y_max, z_max) {
                dirs.push(d - 2 * ds);
            }
        }
        let d1 = Dir::new(d[0].abs() - 1, d[1].abs() - 1, d[2].abs() - 1);
        if has_forceneighbour_check(&(pos + d1), d, occ_set, x_max, y_max, z_max) {
            dirs.push(d + d1);
        }
        if has_forceneighbour_check(&(pos - d1), d, occ_set, x_max, y_max, z_max) {
            dirs.push(d - d1);
        }
    }
    // [go 3D-diagonal]  ----------------------------------------------
    else {
        dirs = get_forceneighbour_dirs(pos, &Dir::new(0, d[1], d[2]), occ_set, x_max, y_max, z_max);
        dirs.append(&mut get_forceneighbour_dirs(
            pos,
            &Dir::new(d[0], 0, d[2]),
            occ_set,
            x_max,
            y_max,
            z_max,
        ));
        dirs.append(&mut get_forceneighbour_dirs(
            pos,
            &Dir::new(d[0], d[1], 0),
            occ_set,
            x_max,
            y_max,
            z_max,
        ));
        dirs = dirs.into_iter().unique().filter(|x| x != d).collect();
    }
    dirs
}

#[inline]
pub fn step(pos: &Grid, d: &Dir) -> Grid {
    pos + d
}

#[allow(unused)]
mod tests {
    use super::dir_2dto1d;
    use crate::linalg::{OVector, U3};
    use crate::{jps_3d_v2, Dir, Grid};
    use std::collections::HashSet;

    #[test]
    #[allow(unused)]
    pub fn test_all_dirs() {
        let mut out: Vec<Dir> = Vec::new();
        let o = Grid::new(0, 0, 0);
        for x in -1..2 {
            for y in -1..2 {
                for z in -1..2 {
                    let p = Grid::new(x, y, z);
                    let dir = o - p;
                    if dir.dot(&dir) == 0 {
                        continue;
                    }
                    println!("Dir::new({},{},{}),", dir[0], dir[1], dir[2]);
                    out.push(dir);
                }
            }
        }
        assert!(out.len() == 26);
    }
    #[test]
    pub fn test_dir_2dto1d() {
        let dir = Dir::new(1, 0, -1);
        let subdirs = dir_2dto1d(&dir);
        println!("{:?}", subdirs);
        assert!(subdirs[0] + subdirs[1] == dir);
    }

    #[test]
    pub fn test_jps_3d() {
        let map = [
            //↓ start
            0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1,
            1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1,
            1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1,
            1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0,
            0, 0, 0, 0, 0, 1, 1, 1, 0, //# ← goal
        ];

        let mut occ_set: HashSet<Grid> = HashSet::new();

        for (b, idx) in map.into_iter().enumerate() {
            if idx == 1 {
                // occupied coordinates
                let z = b as i32 / 25;
                let y = (b as i32 - 25 * z) / 5;
                let x = b as i32 - 25 * z - 5 * y;

                occ_set.insert(Grid::new(x, y, z));
            }
        }

        let binary_map: Vec<bool> = map.into_iter().map(|x| x == 1).collect();
        let map = &binary_map[..];
        let map_start = Grid::new(0, 0, 0);
        let map_goal = Grid::new(4, 4, 4);
        let (x_max, y_max, z_max) = (4, 4, 4);
        let path = jps_3d_v2(&occ_set, &map_start, &map_goal, x_max, y_max, z_max);
        assert!(path.is_some());
        println!("{:?}", path.unwrap());
    }
}
