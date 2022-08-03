# jps : Jump Point Search in Rust.
Jump Point Search Algorithm Implementation in Rust.


# Current implementation status

## JPS Implementation
* 3D case✅ 
## Lifetime 
**Currently no lifetime notation is used, and hence a few unnecessary copies have to be performed. Lifetime is needed to be considered for improving the speed/efficiency**

# Usage
Add this to your Cargo.toml:
```
[dependencies]
jps = "1.0"
```
# Example
see the test
```
cargo test -- --show-output
```
the output shows:
```
path is :
(0, 0, 0) ➡ (0, 1, 0) ➡ (0, 2, 0) ➡ (0, 3, 0) ➡ (1, 4, 1) ➡ (2, 4, 1) ➡ (3, 4, 1) ➡ (4, 3, 2) ➡ (4, 2, 2) ➡ (4, 1, 2) ➡ (3, 0, 3) ➡ (2, 1, 4) ➡ (2, 2, 4) ➡ (3, 3, 4) ➡ (4, 4, 4)
```
# The Map Description
Two version of main API are provided for 3D JPS. The only difference lies in the input parameter, i.e.

## `jps_3d_v1`
`jps_3d_v1` accept a reference of whole map `map: &[bool]`, which is a 0-1 binary reshaped array representing Nx × Ny × Nz 3D map. for example, the following map unfold the 5×5×5 cube into 5 xy planes，each of which has 5×5 grids. 0 indicates free, 1 occupied.
```
// column is x index
//
// z = 0
//↓ start 
0, 1, 1, 1, 1,      // -> y = 0
0, 1, 1, 1, 1,      // -> y = 1
0, 1, 1, 1, 1,      // -> y = 2
0, 1, 1, 1, 1,      // -> y = 3
0, 1, 1, 1, 1,      // -> y = 4

// z = 1
1, 1, 1, 1, 1,  
1, 1, 1, 1, 1,
1, 1, 1, 1, 1, 
1, 1, 1, 1, 1, 
0, 0, 0, 0, 0, 

// z = 2
1, 1, 1, 1, 0, 
1, 1, 1, 1, 0, 
1, 1, 1, 1, 0, 
1, 1, 1, 1, 0, 
1, 1, 1, 1, 0, 

// z = 3
0, 0, 0, 0, 0, 
1, 1, 1, 1, 1, 
1, 1, 1, 1, 1, 
1, 1, 1, 1, 1, 
1, 1, 1, 1, 1, 

// z = 4
0, 1, 1, 1, 1, 
0, 1, 1, 1, 1, 
0, 1, 1, 1, 1, 
0, 0, 0, 0, 0, 
0, 1, 1, 1, 0, // ← goal
```
## `jps_3d_v2`
`jps_3d_v2`expects a reference of only the occuipied HashSet that contains all occupied grids.



# Reference

***Logic follows the C++ version (https://github.com/KumarRobotics/jps3d)***

***Also NOTE that some functions havn't been thoroughly tested, so please let me know if there is any error.***


