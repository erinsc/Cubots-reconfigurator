//! Structs and classes responsible for representing the cubot environment and simulating actions in it.
//! 
//! This module defines the cubot environment, functions for generating and validating cubot actions against the environment. Most functions also keep count of how many operations have been performed, to allow for tracking how effective different algorithms are.
//! 
//! - [Vector] - Struct for representing positions and offsets in an integer grid.
//! - [Action] - Struct representing the Action of a cubot, uniquely defined.
//! - [Statistic] - Struct for recording the number of performed actions.
//! - [Grid] - Struct for representing a set of cubots in a grid, queues Actions for them and checks their validity.

pub mod vector;
#[doc(inline)]
pub use vector::*;
pub mod action;
#[doc(inline)]
pub use action::*;
pub mod grid;
#[doc(inline)]
pub use grid::*;
pub mod statistic;
#[doc(inline)]
pub use statistic::*;