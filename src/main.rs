//! BPPS Testing suite
//! 
//! The purpose of this project is to be a testing suite for different algorithms meant to solve the cubots path planning problem. It was used dynamically to test different features and algorithms and as such has a lot of legacy features that aren't used much anymore.
//! 
//! As this wasn't intended for public viewing, the project does not have a way to be run through the console. To run different algorithms or with different settings, the main function is to be adjusted and the project recompiled. 
//! 
//! # Typical runtime
//! 
//! ```
//! #[tokio::main]
//! pub async fn main() {
//!     // Numbers of agents to test the algorithm against. Should be ordered in ascending order
//!     let agent_counts = vec![
//!         5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
//!         20, 25, 30 ,35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95,
//!         100,
//!     ];
//!     // Algorithm and generator to use, filename to save it as.
//!     let generator = generators::random_walk_equivalent;
//!     let algorithm = algorithms::path_search;
//!     let gen_name = "BPPS Generic";
//!     // Number of seconds and trials to test the algorithm
//!     let time = 60;
//!     let trial_count = 10;
//!
//!     let trials = trial::measure(algorithms::path_search, 
//!                                 generator, 
//!                                 &agent_counts, 
//!                                 time, 
//!                                 trial_count).await;
//!
//!     print_trials(trials, format!("{}", gen_name).as_str());
//! }
//! ```

pub mod environment;
pub mod generators;
pub mod algorithms;
pub mod heuristics;
pub mod utils;
pub mod trial;

#[allow(unused)]
#[doc(inline)]
use crate::{
    environment::*,
    algorithms::*,
    heuristics::*,
    trial::*,
    utils::*
};

/// Main function. To allow premature termination of algorithms, uses the [`tokio`] library.
#[tokio::main]
pub async fn main() {
    #[cfg(debug_assertions)] { println!("Running in Debug mode"); }
    #[cfg(not(debug_assertions))] { println!("Running in Release mode"); }

    let agent_counts = vec![
        5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
        20, 25, 30 ,35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95,
        100,
        ];
    let generator = generators::random_walk_equivalent;
    let gen_name = "Generic";
    let time = 60;
    let trial_count = 10;

    let trials = trial::measure(algorithms::path_search, 
                                 generator, 
                                 &agent_counts, 
                                 time, 
                                 trial_count).await;

    print_trials(trials, format!("BPPS {}", gen_name).as_str());
}
