# jps : Jump Point Search in Rust.
Jump Point Search Algorithm Implementation in Rust.


# Current implementation status

## JPS Implementation
* 3D case✅ 
## Lifetime 
**Currently no lifetime notation is used, and hence a few unnecessary copies have to be performed. Lifetime is needed to be considered for improving the speed/efficiency.**

# Usage
Add this to your Cargo.toml:
```
[dependencies]
jps = "1.1"
```
Two steps to use JPS: 
1) create a `GraphSearch` using either `new_v1` or `new_v2` 
2) call `plan` to plan a path from ***start*** to ***goal***.
```
/* using HashMap as the map input.
`fmap` can be `None` such that a grid will be regarded as free as long as it is not occupied and is within the legal boxed-boundary. */
GraphSearch::new_v1(
        fmap_: Option<HashSet<(i32, i32, i32)>>,
        omap_: HashSet<(i32, i32, i32)>,
        xdim: [i32; 2],
        ydim: [i32; 2],
        zdim: [i32; 2],
        eps_: f32,
    )-> Self

/* using a binary map as the map input */
GraphSearch::new_v2(
        map: &[bool],
        xdim: [i32; 2],
        ydim: [i32; 2],
        zdim: [i32; 2],
        eps_: f32,
        // goal: (i32, i32, i32),
        // use_jps_: bool,
    ) -> Self

/* path-finding */
plan(
        &mut self,
        start: (i32, i32, i32),
        goal: (i32, i32, i32),
        usejps: bool,
        max_expand: i32,
    ) -> bool
```
# Example & Test
```
cargo test -- --show-output
```
the output shows:
```
path is :
(0, 0, 0) ➡ (0, 1, 0)➡ (0, 2, 0)➡ (0, 3, 0)➡ (1, 4, 1)➡ (2, 4, 1)➡ (3, 4, 1)➡ (4, 3, 2)➡ (4, 2, 2)➡ (4, 1, 2)➡ (3, 0, 3)➡ (2, 1, 4)➡ (2, 2, 4)➡ (3, 3, 4)➡ (4, 4, 4)
turnings are :
(0, 0, 0) (0, 3, 0) (1, 4, 1) (3, 4, 1) (4, 3, 2) (4, 1, 2) (3, 0, 3) (2, 1, 4) (2, 2, 4) (4, 4, 4)
```
# Release Note
## Ver 1.1.0
- API changed. 

    `xdim : i32` is replaced by `xbound: [i32;2]`. `xbound[0] ≤ x < xbound[1]` defines the x-axis grid searching space. (similar case for `ydim` and `zdim`)
- Add a functional feature, namely **`decomp`**, for finding ellipsoid and decomposition.

    To use this feature, in Cargo.toml add
    ```
    [dependencies]
    jps = { version = "1.1", features = ["decomp"] }
    ```
    You need to implement trait `Voxelable` for voxelization and devoxelization. 
    
    Then you can `init` a **`Decomp`**, and `decompose` it to perform the decomposition. 

    As a simple example, check the example in the test, or type:
    ```
    cargo test --release --features=decomp -- --show-output
    ```


# Reference

***Logic follows the C++ version (https://github.com/KumarRobotics/jps3d)***

***Also NOTE that some functions havn't been thoroughly tested, so please let me know if there is any error.***


