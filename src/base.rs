use ordered_float::OrderedFloat;
use std::f32::INFINITY;

/// Node of the graph in graph search
#[derive(Debug, Hash, Eq, PartialEq, Copy, Clone)]
pub struct State {
    /// ID
    pub id: (i32, i32, i32),
    ///Coord
    pub x: i32,
    pub y: i32,
    pub z: i32,
    /// direction
    pub dx: i32,
    pub dy: i32,
    pub dz: i32,
    /// id of predicessors
    pub parent_id: Option<(i32, i32, i32)>,
    /// g cost
    pub g: OrderedFloat<f32>,
    /// heuristic cost
    pub h: OrderedFloat<f32>,
    /// if has been opened
    pub opened: bool,
    /// if has been closed
    pub closed: bool,
}
impl Default for State {
    fn default() -> Self {
        Self {
            id: (0, 0, 0),
            x: 0,
            y: 0,
            z: 0,
            dx: 0,
            dy: 0,
            dz: 0,
            parent_id: None,
            g: OrderedFloat(INFINITY),
            h: OrderedFloat(INFINITY),
            opened: false,
            closed: false,
        }
    }
}

impl State {
    pub fn new(id: (i32, i32, i32), x: i32, y: i32, z: i32, dx: i32, dy: i32, dz: i32) -> Self {
        Self {
            id: id,
            x: x,
            y: y,
            z: z,
            dx: dx,
            dy: dy,
            dz: dz,
            ..Default::default()
        }
    }
}

#[derive(Debug)]
/// Search and prune neighbors for JPS 3D
pub struct JPS3DNeib {
    // for each (dx,dy,dz) these contain:
    //    ns: neighbors that are always added
    //    f1: forced neighbors to check
    //    f2: neighbors to add if f1 is forced
    pub ns: [[[i32; 26]; 3]; 27],
    pub f1: [[[i32; 12]; 3]; 27],
    pub f2: [[[i32; 12]; 3]; 27],
}

impl Default for JPS3DNeib {
    fn default() -> Self {
        let mut jps3dneib = Self {
            ns: [[[0; 26]; 3]; 27],
            f1: [[[0; 12]; 3]; 27],
            f2: [[[0; 12]; 3]; 27],
        };
        let mut id: usize = 0;
        for dz in -1..2_i32 {
            for dy in -1..2_i32 {
                for dx in -1..2_i32 {
                    let norm1 = dx.abs() + dy.abs() + dz.abs();
                    for dev in 0..Self::nsz[norm1 as usize][0] {
                        JPS3DNeib::Neib(dx, dy, dz, norm1, dev, id, &mut jps3dneib.ns);
                    }
                    for dev in 0..Self::nsz[norm1 as usize][1] {
                        JPS3DNeib::FNeib(
                            dx,
                            dy,
                            dz,
                            norm1,
                            dev,
                            id,
                            &mut jps3dneib.f1,
                            &mut jps3dneib.f2,
                        );
                    }
                    id += 1;
                }
            }
        }
        jps3dneib
    }
}

impl JPS3DNeib {
    // nsz contains the number of neighbors for the four different types of moves:
    // no move (norm 0):        26 neighbors always added
    //                          0 forced neighbors to check (never happens)
    //                          0 neighbors to add if forced (never happens)
    // straight (norm 1):       1 neighbor always added
    //                          8 forced neighbors to check
    //                          8 neighbors to add if forced
    // diagonal (norm sqrt(2)): 3 neighbors always added
    //                          8 forced neighbors to check
    //                          12 neighbors to add if forced
    // diagonal (norm sqrt(3)): 7 neighbors always added
    //                          6 forced neighbors to check
    //                          12 neighbors to add if forced
    #[allow(non_upper_case_globals)]
    pub const nsz: [[i32; 2]; 4] = [[26, 0], [1, 8], [3, 12], [7, 12]];

    #[allow(non_snake_case)]
    pub(crate) fn Neib(
        dx: i32,
        dy: i32,
        dz: i32,
        norm1: i32,
        dev: i32,
        id: usize,
        t: &mut [[[i32; 26]; 3]; 27],
    ) {
        let (mut x, mut y, mut z) = (0, 0, 0);
        let tx: &mut i32 = &mut x;
        let ty: &mut i32 = &mut y;
        let tz: &mut i32 = &mut z;

        match norm1 {
            // norm1 == 0
            0 => match dev {
                0 => {
                    *tx = 1;
                    *ty = 0;
                    *tz = 0;
                }
                1 => {
                    *tx = -1;
                    *ty = 0;
                    *tz = 0;
                }
                2 => {
                    *tx = 0;
                    *ty = 1;
                    *tz = 0;
                }
                3 => {
                    *tx = 1;
                    *ty = 1;
                    *tz = 0;
                }
                4 => {
                    *tx = -1;
                    *ty = 1;
                    *tz = 0;
                }
                5 => {
                    *tx = 0;
                    *ty = -1;
                    *tz = 0;
                }
                6 => {
                    *tx = 1;
                    *ty = -1;
                    *tz = 0;
                }
                7 => {
                    *tx = -1;
                    *ty = -1;
                    *tz = 0;
                }
                8 => {
                    *tx = 0;
                    *ty = 0;
                    *tz = 1;
                }
                9 => {
                    *tx = 1;
                    *ty = 0;
                    *tz = 1;
                }
                10 => {
                    *tx = -1;
                    *ty = 0;
                    *tz = 1;
                }
                11 => {
                    *tx = 0;
                    *ty = 1;
                    *tz = 1;
                }
                12 => {
                    *tx = 1;
                    *ty = 1;
                    *tz = 1;
                }
                13 => {
                    *tx = -1;
                    *ty = 1;
                    *tz = 1;
                }
                14 => {
                    *tx = 0;
                    *ty = -1;
                    *tz = 1;
                }
                15 => {
                    *tx = 1;
                    *ty = -1;
                    *tz = 1;
                }
                16 => {
                    *tx = -1;
                    *ty = -1;
                    *tz = 1;
                }
                17 => {
                    *tx = 0;
                    *ty = 0;
                    *tz = -1;
                }
                18 => {
                    *tx = 1;
                    *ty = 0;
                    *tz = -1;
                }
                19 => {
                    *tx = -1;
                    *ty = 0;
                    *tz = -1;
                }
                20 => {
                    *tx = 0;
                    *ty = 1;
                    *tz = -1;
                }
                21 => {
                    *tx = 1;
                    *ty = 1;
                    *tz = -1;
                }
                22 => {
                    *tx = -1;
                    *ty = 1;
                    *tz = -1;
                }
                23 => {
                    *tx = 0;
                    *ty = -1;
                    *tz = -1;
                }
                24 => {
                    *tx = 1;
                    *ty = -1;
                    *tz = -1;
                }
                25 => {
                    *tx = -1;
                    *ty = -1;
                    *tz = -1;
                }
                _ => {
                    panic!("Shouldn't happen");
                }
            },
            // norm1 == 1
            1 => {
                *tx = dx;
                *ty = dy;
                *tz = dz;
            }
            // norm1 == 2
            2 => match dev {
                0 => {
                    if dz == 0 {
                        *tx = 0;
                        *ty = dy;
                        *tz = 0;
                    } else {
                        *tx = 0;
                        *ty = 0;
                        *tz = dz;
                    }
                }
                1 => {
                    if dx == 0 {
                        *tx = 0;
                        *ty = dy;
                        *tz = 0;
                    } else {
                        *tx = dx;
                        *ty = 0;
                        *tz = 0;
                    }
                }
                2 => {
                    *tx = dx;
                    *ty = dy;
                    *tz = dz;
                }
                _ => panic!("Shouln't happen"),
            },
            // norm1 == 3
            3 => match dev {
                0 => {
                    *tx = dx;
                    *ty = 0;
                    *tz = 0;
                }
                1 => {
                    *tx = 0;
                    *ty = dy;
                    *tz = 0;
                }
                2 => {
                    *tx = 0;
                    *ty = 0;
                    *tz = dz;
                }
                3 => {
                    *tx = dx;
                    *ty = dy;
                    *tz = 0;
                }
                4 => {
                    *tx = dx;
                    *ty = 0;
                    *tz = dz;
                }
                5 => {
                    *tx = 0;
                    *ty = dy;
                    *tz = dz;
                }
                6 => {
                    *tx = dx;
                    *ty = dy;
                    *tz = dz;
                }
                _ => panic!("Shouldn't happen"),
            },
            // other norm1
            _ => panic!("Shouldn't happen"),
        }
        // apply changes
        t[id][0][dev as usize] = x;
        t[id][1][dev as usize] = y;
        t[id][2][dev as usize] = z;
        // *(&mut (t[id][0][dev as usize])) = x;
        // *(&mut (t[id][1][dev as usize])) = y;
        // *(&mut (t[id][2][dev as usize])) = z;
    }

    #[allow(non_snake_case)]
    pub(crate) fn FNeib(
        dx: i32,
        dy: i32,
        dz: i32,
        norm1: i32,
        dev: i32,
        id: usize,
        f1: &mut [[[i32; 12]; 3]; 27],
        f2: &mut [[[i32; 12]; 3]; 27],
    ) {
        let (mut x, mut y, mut z, mut xx, mut yy, mut zz) = (0, 0, 0, 0, 0, 0);
        let fx: &mut i32 = &mut x;
        let fy: &mut i32 = &mut y;
        let fz: &mut i32 = &mut z;
        let nx: &mut i32 = &mut xx;
        let ny: &mut i32 = &mut yy;
        let nz: &mut i32 = &mut zz;

        match norm1 {
            // norm1 == 1
            1 => {
                match dev {
                    0 => {
                        *fx = 0;
                        *fy = 1;
                        *fz = 0;
                    }
                    1 => {
                        *fx = 0;
                        *fy = -1;
                        *fz = 0;
                    }
                    2 => {
                        *fx = 1;
                        *fy = 0;
                        *fz = 0;
                    }
                    3 => {
                        *fx = 1;
                        *fy = 1;
                        *fz = 0;
                    }
                    4 => {
                        *fx = 1;
                        *fy = -1;
                        *fz = 0;
                    }
                    5 => {
                        *fx = -1;
                        *fy = 0;
                        *fz = 0;
                    }
                    6 => {
                        *fx = -1;
                        *fy = 1;
                        *fz = 0;
                    }
                    7 => {
                        *fx = -1;
                        *fy = -1;
                        *fz = 0;
                    }
                    _ => panic!("Shouldn't happen"),
                }
                *nx = *fx;
                *ny = *fy;
                *nz = dz;
                // switch order if different direction
                if dx != 0 {
                    *fz = *fx;
                    *fx = 0;
                    *nz = *fz;
                    *nx = dx;
                }
                if dy != 0 {
                    *fz = *fy;
                    *fy = 0;
                    *nz = *fz;
                    *ny = dy;
                }
            }
            // norm1 == 2
            2 => {
                if dx == 0 {
                    match dev {
                        0 => {
                            *fx = 0;
                            *fy = 0;
                            *fz = -dz;
                            *nx = 0;
                            *ny = dy;
                            *nz = -dz;
                        }
                        1 => {
                            *fx = 0;
                            *fy = -dy;
                            *fz = 0;
                            *nx = 0;
                            *ny = -dy;
                            *nz = dz;
                        }
                        2 => {
                            *fx = 1;
                            *fy = 0;
                            *fz = 0;
                            *nx = 1;
                            *ny = dy;
                            *nz = dz;
                        }
                        3 => {
                            *fx = -1;
                            *fy = 0;
                            *fz = 0;
                            *nx = -1;
                            *ny = dy;
                            *nz = dz;
                        }
                        4 => {
                            *fx = 1;
                            *fy = 0;
                            *fz = -dz;
                            *nx = 1;
                            *ny = dy;
                            *nz = -dz;
                        }
                        5 => {
                            *fx = 1;
                            *fy = -dy;
                            *fz = 0;
                            *nx = 1;
                            *ny = -dy;
                            *nz = dz;
                        }
                        6 => {
                            *fx = -1;
                            *fy = 0;
                            *fz = -dz;
                            *nx = -1;
                            *ny = dy;
                            *nz = -dz;
                        }
                        7 => {
                            *fx = -1;
                            *fy = -dy;
                            *fz = 0;
                            *nx = -1;
                            *ny = -dy;
                            *nz = dz;
                        }
                        // Extras
                        8 => {
                            *fx = 1;
                            *fy = 0;
                            *fz = 0;
                            *nx = 1;
                            *ny = dy;
                            *nz = 0;
                        }
                        9 => {
                            *fx = 1;
                            *fy = 0;
                            *fz = 0;
                            *nx = 1;
                            *ny = 0;
                            *nz = dz;
                        }
                        10 => {
                            *fx = -1;
                            *fy = 0;
                            *fz = 0;
                            *nx = -1;
                            *ny = dy;
                            *nz = 0;
                        }
                        11 => {
                            *fx = -1;
                            *fy = 0;
                            *fz = 0;
                            *nx = -1;
                            *ny = 0;
                            *nz = dz;
                        }
                        _ => panic!("Shouldn't happen"),
                    }
                } else if dy == 0 {
                    match dev {
                        0 => {
                            *fx = 0;
                            *fy = 0;
                            *fz = -dz;
                            *nx = dx;
                            *ny = 0;
                            *nz = -dz;
                        }
                        1 => {
                            *fx = -dx;
                            *fy = 0;
                            *fz = 0;
                            *nx = -dx;
                            *ny = 0;
                            *nz = dz;
                        }
                        2 => {
                            *fx = 0;
                            *fy = 1;
                            *fz = 0;
                            *nx = dx;
                            *ny = 1;
                            *nz = dz;
                        }
                        3 => {
                            *fx = 0;
                            *fy = -1;
                            *fz = 0;
                            *nx = dx;
                            *ny = -1;
                            *nz = dz;
                        }
                        4 => {
                            *fx = 0;
                            *fy = 1;
                            *fz = -dz;
                            *nx = dx;
                            *ny = 1;
                            *nz = -dz;
                        }
                        5 => {
                            *fx = -dx;
                            *fy = 1;
                            *fz = 0;
                            *nx = -dx;
                            *ny = 1;
                            *nz = dz;
                        }
                        6 => {
                            *fx = 0;
                            *fy = -1;
                            *fz = -dz;
                            *nx = dx;
                            *ny = -1;
                            *nz = -dz;
                        }
                        7 => {
                            *fx = -dx;
                            *fy = -1;
                            *fz = 0;
                            *nx = -dx;
                            *ny = -1;
                            *nz = dz;
                        }
                        // Extras
                        8 => {
                            *fx = 0;
                            *fy = 1;
                            *fz = 0;
                            *nx = dx;
                            *ny = 1;
                            *nz = 0;
                        }
                        9 => {
                            *fx = 0;
                            *fy = 1;
                            *fz = 0;
                            *nx = 0;
                            *ny = 1;
                            *nz = dz;
                        }
                        10 => {
                            *fx = 0;
                            *fy = -1;
                            *fz = 0;
                            *nx = dx;
                            *ny = -1;
                            *nz = 0;
                        }
                        11 => {
                            *fx = 0;
                            *fy = -1;
                            *fz = 0;
                            *nx = 0;
                            *ny = -1;
                            *nz = dz;
                        }
                        _ => panic!("Shouldn't happen"),
                    }
                } else {
                    // dz == 0
                    match dev {
                        0 => {
                            *fx = 0;
                            *fy = -dy;
                            *fz = 0;
                            *nx = dx;
                            *ny = -dy;
                            *nz = 0;
                        }
                        1 => {
                            *fx = -dx;
                            *fy = 0;
                            *fz = 0;
                            *nx = -dx;
                            *ny = dy;
                            *nz = 0;
                        }
                        2 => {
                            *fx = 0;
                            *fy = 0;
                            *fz = 1;
                            *nx = dx;
                            *ny = dy;
                            *nz = 1;
                        }
                        3 => {
                            *fx = 0;
                            *fy = 0;
                            *fz = -1;
                            *nx = dx;
                            *ny = dy;
                            *nz = -1;
                        }
                        4 => {
                            *fx = 0;
                            *fy = -dy;
                            *fz = 1;
                            *nx = dx;
                            *ny = -dy;
                            *nz = 1;
                        }
                        5 => {
                            *fx = -dx;
                            *fy = 0;
                            *fz = 1;
                            *nx = -dx;
                            *ny = dy;
                            *nz = 1;
                        }
                        6 => {
                            *fx = 0;
                            *fy = -dy;
                            *fz = -1;
                            *nx = dx;
                            *ny = -dy;
                            *nz = -1;
                        }
                        7 => {
                            *fx = -dx;
                            *fy = 0;
                            *fz = -1;
                            *nx = -dx;
                            *ny = dy;
                            *nz = -1;
                        }
                        // Extras
                        8 => {
                            *fx = 0;
                            *fy = 0;
                            *fz = 1;
                            *nx = dx;
                            *ny = 0;
                            *nz = 1;
                        }
                        9 => {
                            *fx = 0;
                            *fy = 0;
                            *fz = 1;
                            *nx = 0;
                            *ny = dy;
                            *nz = 1;
                        }
                        10 => {
                            *fx = 0;
                            *fy = 0;
                            *fz = -1;
                            *nx = dx;
                            *ny = 0;
                            *nz = -1;
                        }
                        11 => {
                            *fx = 0;
                            *fy = 0;
                            *fz = -1;
                            *nx = 0;
                            *ny = dy;
                            *nz = -1;
                        }
                        _ => panic!("Shouldn't happen"),
                    }
                }
            }
            // norm1 == 3
            3 => {
                match dev {
                    0 => {
                        *fx = -dx;
                        *fy = 0;
                        *fz = 0;
                        *nx = -dx;
                        *ny = dy;
                        *nz = dz;
                    }
                    1 => {
                        *fx = 0;
                        *fy = -dy;
                        *fz = 0;
                        *nx = dx;
                        *ny = -dy;
                        *nz = dz;
                    }
                    2 => {
                        *fx = 0;
                        *fy = 0;
                        *fz = -dz;
                        *nx = dx;
                        *ny = dy;
                        *nz = -dz;
                    }
                    // Need to check up to here for forced!
                    3 => {
                        *fx = 0;
                        *fy = -dy;
                        *fz = -dz;
                        *nx = dx;
                        *ny = -dy;
                        *nz = -dz;
                    }
                    4 => {
                        *fx = -dx;
                        *fy = 0;
                        *fz = -dz;
                        *nx = -dx;
                        *ny = dy;
                        *nz = -dz;
                    }
                    5 => {
                        *fx = -dx;
                        *fy = -dy;
                        *fz = 0;
                        *nx = -dx;
                        *ny = -dy;
                        *nz = dz;
                    }
                    // Extras
                    6 => {
                        *fx = -dx;
                        *fy = 0;
                        *fz = 0;
                        *nx = -dx;
                        *ny = 0;
                        *nz = dz;
                    }
                    7 => {
                        *fx = -dx;
                        *fy = 0;
                        *fz = 0;
                        *nx = -dx;
                        *ny = dy;
                        *nz = 0;
                    }
                    8 => {
                        *fx = 0;
                        *fy = -dy;
                        *fz = 0;
                        *nx = 0;
                        *ny = -dy;
                        *nz = dz;
                    }
                    9 => {
                        *fx = 0;
                        *fy = -dy;
                        *fz = 0;
                        *nx = dx;
                        *ny = -dy;
                        *nz = 0;
                    }
                    10 => {
                        *fx = 0;
                        *fy = 0;
                        *fz = -dz;
                        *nx = 0;
                        *ny = dy;
                        *nz = -dz;
                    }
                    11 => {
                        *fx = 0;
                        *fy = 0;
                        *fz = -dz;
                        *nx = dx;
                        *ny = 0;
                        *nz = -dz;
                    }
                    _ => panic!("Shouldn't happen"),
                }
            }
            // norm1 other cases
            _ => panic!("Shouldn't happen"),
        }

        f1[id][0][dev as usize] = x;
        f1[id][1][dev as usize] = y;
        f1[id][2][dev as usize] = z;
        f2[id][0][dev as usize] = xx;
        f2[id][1][dev as usize] = yy;
        f2[id][2][dev as usize] = zz;
    }
}
