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
jps = "1.0"
```
# Example & Test
```
cargo test -- --show-output
```
the output shows:
```
path is :
(0, 0, 0) ➡ (0, 1, 0) ➡ (0, 2, 0) ➡ (0, 3, 0) ➡ (1, 4, 1) ➡ (2, 4, 1) ➡ (3, 4, 1) ➡ (4, 3, 2) ➡ (4, 2, 2) ➡ (4, 1, 2) ➡ (3, 0, 3) ➡ (2, 1, 4) ➡ (2, 2, 4) ➡ (3, 3, 4) ➡ (4, 4, 4)
```


# Reference

***Logic follows the C++ version (https://github.com/KumarRobotics/jps3d)***

***Also NOTE that some functions havn't been thoroughly tested, so please let me know if there is any error.***


