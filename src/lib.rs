// extern crate alloc;
// extern crate float_ord;
extern crate im;
extern crate ordered_float;
extern crate priority_queue;

pub mod base;
pub use graphsearch::GraphSearch;

#[cfg(feature = "decomp")]
pub use decomposition::base::decomp_tool::{self, Decomp, Grp3, Vec3, Voxelable};

mod decomposition;
mod graphsearch;
mod tests;
