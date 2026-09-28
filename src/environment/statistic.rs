//! Struct for tracking the number of operations performed during the runtime of an algorithm.

use std::ops::AddAssign;

/// Struct for recording various statistics about actions an algorithm performs.
/// A single instance is meant to be passed to all methods and functions ran during the algorithm, but adition of statistics is also defined if necessary.
/// Defines methods for adding to all statistics.
/// 
/// # Example
/// 
/// ```
/// let a = Statistic::new();
/// a.add_states_processed(5);
/// a.add_states_processed(7);
/// assert_eq!(a.states_processed, 12);
/// ```
#[derive(Debug, Default, Clone, Copy)]
pub struct Statistic {
    pub states_seen: usize,     // max states kept in memory at once
    pub states_processed: usize, // total states opened during search
    pub frontier_size: usize,    // max frontier size
    pub actions_tested: usize,   // number of actions tested to be valid
    pub grid_actions: usize,     // number of search, insertion, removal on structures
    pub grid_comparisons: usize  // number of structure comparisons
}
impl AddAssign for Statistic {
    fn add_assign(&mut self, rhs: Self) {
        self.states_seen = self.states_seen.max(rhs.states_seen);
        self.states_processed += rhs.states_processed;
        self.frontier_size = self.frontier_size.max(rhs.frontier_size);
        self.actions_tested += rhs.actions_tested;
        self.grid_actions += rhs.grid_actions;
        self.grid_comparisons += rhs.grid_comparisons;
        
    }
}
impl Statistic {
    /// Creates new statistic
    pub fn new() -> Self {
        Statistic::default()
    }
    pub fn max_states_seen(&mut self, states: usize) {
        self.states_seen = self.states_seen.max(states);
    }
    pub fn add_states_processed(&mut self, states: usize) {
        self.states_processed += states;
    }
    pub fn max_frontier_size(&mut self, size: usize) {
        self.frontier_size = self.frontier_size.max(size);
    }
    pub fn add_actions_tested(&mut self, actions: usize) {
        self.actions_tested += actions;
    }
    pub fn add_grid_actions(&mut self, grids: usize) {
        self.grid_actions += grids;
    }
    pub fn add_grid_comparisons(&mut self, grids: usize) {
        self.grid_comparisons += grids;
    }
}
impl std::fmt::Display for Statistic {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "Statistics: \n")?;
        write!(f, "  States seen: {}\n", self.states_seen)?;
        write!(f, "  States processed: {}\n", self.states_processed)?;
        write!(f, "  Frontier size: {}\n", self.frontier_size)?;
        write!(f, "  Actions tested: {}\n", self.actions_tested)?;
        write!(f, "  Grid actions: {}\n", self.grid_actions)?;
        write!(f, "  Grid comparisons: {}\n", self.grid_comparisons)?;
        Ok(())
    }
}