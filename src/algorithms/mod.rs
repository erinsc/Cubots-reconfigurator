//! Path-finding algorithms for cubots.
//! 
//! This module contains all the implemented algorithms for solving the cubot problem. It also contains other helping search algorithms which may be used within the others.
//!
//! - [find_path()] - Plans a path for a single cubot. 
//! - [astar_closest_canon()], [astar_hungarian_canon()], [hstar_canon()] - Different variations of A*.
//! - [path_search()] - BPPS algorithm.
//! - [pp_search()] - Priority Planning algorithm. Finds an ordering of cubots in which it plans a dissasembly. Not complete
//! - [removeable()] - Checks if its possible to remove a cubot from a structure and have it remain connected.

#![allow(unused)]
pub mod removeable;
#[doc(inline)]
pub use removeable::removeable as removeable;
pub mod astar_closest;
#[doc(inline)]
pub use astar_closest::astar_closest_canon as astar_closest_canon;
pub mod astar_hungarian;
#[doc(inline)]
pub use astar_hungarian::astar_hungarian_canon as astar_hungarian_canon;
pub mod hstar;
#[doc(inline)]
pub use hstar::hstar_canon as hstar_canon;
pub mod find_path;
#[doc(inline)]
pub use find_path::find_path as find_path;
pub mod path_search;
#[doc(inline)]
pub use path_search::backtrack_path_search as path_search;
pub mod pp_search;
#[doc(inline)]
pub use pp_search::prioritized_planning_search as pp_search;
