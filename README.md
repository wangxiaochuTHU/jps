# jps : Jump Point Search in Rust.
Jump Point Search Algorithm Implementation in Rust.

**NOTE (Very Important)**: 

This Crate currently has bugs such that it would only work in some special scenes that require plenties of grids with force neighbors due to the incomplete recursion. So DON'T use it for important work. Next I will take time to read the C++ version and try to follow it.





# Current implementation status

## JPS Implementation
* 3D case✅ 

# Usage
Add this to your Cargo.toml:
```
[dependencies]
jps = "0.1"
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

***Logic follows the Matlab version (https://github.com/LenaShengzhen/AerialRobotics)***

***Also NOTE that some functions havn't been thoroughly tested, so please let me know if there is any error.***


