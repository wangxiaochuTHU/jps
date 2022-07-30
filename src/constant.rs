use crate::linalg::{OVector, U3};

pub type Dir = OVector<i32, U3>;

#[allow(unused)]
pub const ALL_DIRS: [Dir; 26] = [
    Dir::new(1, 1, 1),
    Dir::new(1, 1, 0),
    Dir::new(1, 1, -1),
    Dir::new(1, 0, 1),
    Dir::new(1, 0, 0),
    Dir::new(1, 0, -1),
    Dir::new(1, -1, 1),
    Dir::new(1, -1, 0),
    Dir::new(1, -1, -1),
    Dir::new(0, 1, 1),
    Dir::new(0, 1, 0),
    Dir::new(0, 1, -1),
    Dir::new(0, 0, 1),
    Dir::new(0, 0, -1),
    Dir::new(0, -1, 1),
    Dir::new(0, -1, 0),
    Dir::new(0, -1, -1),
    Dir::new(-1, 1, 1),
    Dir::new(-1, 1, 0),
    Dir::new(-1, 1, -1),
    Dir::new(-1, 0, 1),
    Dir::new(-1, 0, 0),
    Dir::new(-1, 0, -1),
    Dir::new(-1, -1, 1),
    Dir::new(-1, -1, 0),
    Dir::new(-1, -1, -1),
];
