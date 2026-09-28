//! Struct for representing the state of a set of cubots in the same grid.
//! 
//! The grid stores the positions and planned actions of all robots in the grid.
//! It's also responsible for validating and checking if actions are possible, aswell as simulating actions being performed.

use crate::environment::*;
use im::{OrdSet, OrdMap};

pub type Plan = Vec<Action>; // A sequence of actions to perform
pub type ParallelPlan = Vec<Vec<Action>>; // A sequence of sets of actions to perform in parallel

/// Custom errors for queueing actions, used for validation and collision detection in algorithms:
/// - [QueueResult::Ok] - normal response.
/// - [QueueResult::Missing] - The cubot the intended action is for isn't present.
/// - [QueueResult::Reserved] - The action would make the cubot pass through a position which is already reserved by another cubot. It's starting position is returned. If there are multiple, the first is returned.
/// - [QueueResult::Obstructed] - The action would make the cubot pass through a position currently taken by another cubot. This position is returned. If there are multiple, the first is returned.
/// - [QueueResult::NotSolid] - Atleast one of the robots that need to not move has a moving action planned.
#[allow(unused)]
#[derive(PartialEq, Eq, Debug)]
pub enum QueueResult {
    Ok,
    Missing,
    Reserved(Vector),
    Obstructed(Vector),
    NotSolid(Vector)
}

/// Struct for representing the state of a set of cubots in the same grid.
/// 
/// Methods of the struct take a mutable Statistic struct, which tracks the number of grid operations and other values for algorithm efectivity.
/// Typically you should create the grid, populate it with cubots, then only manipulate it using [Action]s. However inserting or removing cubots into and from the grid is allowed at any point, in case an algorithm desires it.
/// 
/// # Example
/// 
/// ```
/// let mut stat = Statistic::new();
/// let mut g = Grid::new();
/// for i in 0..3 {
///     g.insert(Vector::new(i, 0, 0), &mut stat);
/// }
/// let actions = g.possible_actions(&Vector::new(1, 1, 0), &mut stat);
/// let res = g.queue_verbose(actions[0], &mut stat);
/// assert_eq!(res, QueueResult::Ok);
/// ```
#[derive(Clone, Default)]
pub struct Grid {
    pub robots: OrdSet<Vector>, // Robots
    pub reserved: OrdMap<Vector, Vector>, // Positions that a robot is planning to move through
    pub solid: OrdSet<Vector>, // Robots that shall not move
    pub queued: Vec<Action> // Queued actions
}
impl Grid {
    /// Creates a new, empty grid.
    pub fn new() -> Self {
        Self::default()
    }
    /// Returns the number of cubots in the grid.
    pub fn count(&self) -> usize {
        self.robots.len()
    }
    /// Returns the corners of the smallest bounding cuboid containing all cubots.
    pub fn bounds(&self) -> (Vector, Vector) {
        self.iter()
            .fold((Vector::unit(i16::MAX), Vector::unit(i16::MIN)), |(min, max), n| {
            (min.min_bound(n), max.max_bound(n))
        })
    }
    /// Returns an iterator going through all cubots.
    /// The order is lexicographic, with ordering `(y, x, z)`.
    pub fn iter(&self) -> impl Iterator<Item = &Vector> {
        self.robots.iter()
    }
    /// Inserts a new cubot into the grid.
    /// Returns Some<_> if the position was already taken.
    pub fn insert(&mut self, position: Vector, stat: &mut Statistic) -> Option<Vector> {
        stat.add_grid_actions(1);
        self.robots.insert(position)
    }
    /// Removes a cubot from the grid.
    /// Returns None if the position was already empty.
    pub fn remove(&mut self, position: &Vector, stat: &mut Statistic) -> Option<Vector> {
        stat.add_grid_actions(1);
        self.robots.remove(position)
    }
    /// Returns if a position contains a cubot.
    pub fn contains(&self, position: &Vector, stat: &mut Statistic) -> bool {
        stat.add_grid_actions(1);
        self.robots.contains(position)
    }
    /// Returns if a position contains a cubot that doesn't have an action planned.
    pub fn is_stable(&self, position: &Vector, stat: &mut Statistic) -> bool {
        stat.add_grid_actions(1);
        self.contains(position, stat) && 
        !self.reserved.contains_key(position)
    }
    /// Returns if a position is empty and above the ground plane.
    pub fn is_free(&self, position: &Vector, stat: &mut Statistic) -> bool {
        stat.add_grid_actions(1);
        !self.reserved.contains_key(position) && 
        !self.contains(position, stat) && 
        position.y >= 0
    }
    /// Instantly performs a cubot action without checking if it's valid.
    pub fn perform_unchecked(&mut self, action: Action, stat: &mut Statistic) {
        self.remove(&action.start(), stat);
        self.insert(action.end(), stat);
    }
    /// Queues a cubot action without checking if it's valid.
    pub fn queue_unchecked(&mut self, action: Action, stat: &mut Statistic) {
        self.queued.push(action);
        for pos in action.reserved_positions() {
            self.reserved.insert(pos, action.start());
            stat.add_grid_actions(1);
        }
        for pos in action.solid_positions() {
            self.solid.insert(pos);
            stat.add_grid_actions(1);
        }
    }
    /// Queues a cubot action, returning an error if one occured.
    #[allow(unused)]
    pub fn queue_verbose(&mut self, action: Action, stat: &mut Statistic) -> QueueResult {
        use QueueResult::*;
        if !self.contains(&action.start(), stat) {
            return Missing;
        }
        for pos in &action.reserved_positions() {
            stat.add_grid_actions(1);

            if let Some(&v) = self.reserved.get(pos) {
                return Reserved(v);
            }
            if self.contains(pos, stat) && pos != &action.start() {
                return Obstructed(*pos);
            }
        }
        for pos in &action.solid_positions() {
            stat.add_grid_actions(1);

            if self.reserved.get(pos).is_some() ||
               !self.contains(pos, stat) {
                return NotSolid(*pos);
            }
        }
        self.queue_unchecked(action, stat);
        return Ok;
    }
    /// Queues a cubot action, returning `false` if an error occured.
    #[allow(unused)]
    pub fn queue(&mut self, action: Action, stat: &mut Statistic) -> bool {
        if !self.contains(&action.start(), stat) {
            return false;
        }
        for pos in &action.reserved_positions() {
            stat.add_grid_actions(1);

            if self.reserved.get(pos).is_some() ||
               (self.contains(pos, stat) && pos != &action.start()) {
                return false;
            }
        }
        for pos in &action.solid_positions() {
            stat.add_grid_actions(1);

            if self.reserved.get(pos).is_some() ||
               !self.contains(pos, stat) {
                return false;
            }
        }
        self.queue_unchecked(action, stat);
        return true;
    }
    /// Executes all actions in the queue without checking if they're valid.
    pub fn execute_queue(&mut self, stat: &mut Statistic) {
        for act in self.queued.drain(..) {
            stat.add_grid_actions(2);
            self.robots.remove(&act.start());
            self.robots.insert(act.end());
        }
        self.reserved.clear();
        self.solid.clear();
    }
    /// Given a position, returns all actions which are valid in this position.
    /// Does not return a non-moving action, as those are always possible if a cubot is present.
    pub fn possible_actions(&self, position: &Vector, stat: &mut Statistic) -> Vec<Action> {
        use UnitVector::*;
        [Pz, Nz, Px, Nx]
            .into_iter()
            .map(|axis| self.check_rotation(position, axis, stat))
            .filter(|a| !a.is_none())
            .collect()
    }
    /// Given a position and an axis of rotation, calculates the initial direction and action size.
    /// There will be at most one option of action. If it's valid, its returned, otherwise a non-moving action is returned.
    pub fn check_rotation(&self, position: &Vector, axis: UnitVector, stat: &mut Statistic) -> Action {
        use std::iter::successors;
        let pos = position;

        successors(Some(axis.permute()), |up| Some(up.cross(axis)))
                .take(4)
                .filter(|&up| self.is_stable(&(pos - up), stat))
                .collect::<Vec<_>>().into_iter()
                .map(|up| self.check_action(pos, axis, up, stat))
                .find(|a| !a.is_none())
                .unwrap_or(Action::none(*position))
    }
    // Private methods, given a position, axis of rotation, and direction, checks which type of action is possible.
    fn check_action(&self, pos: &Vector, axis: UnitVector, up: UnitVector, stat: &mut Statistic) -> Action {
        stat.add_actions_tested(1);
        let dir = up.cross(axis);

        // Check above
        if !self.is_free(&(pos + up), stat) {
            return Action::none(*pos);
        }
        // Type Jump
        if self.is_stable(&(pos + up + dir), stat) {
            //println!("Type D");
            if self.is_free(&(pos - dir), stat) &&
               self.is_free(&(pos - dir + up), stat) {
                return Action::new(ActionMode::Jump, *pos, axis, up);
               }
            return Action::none(*pos);
        }
        if !self.is_free(&(pos + up + dir), stat) ||
           !self.is_free(&(pos + dir), stat) {
            //println!("Check 1");
            return Action::none(*pos);
        }
        // Type Flat
        if self.is_stable(&(pos + dir - up), stat) {
            return Action::new(ActionMode::Flat, *pos, axis, dir);
        }
        // Type Wall
        if self.is_stable(&(pos + dir + dir), stat) {
            return Action::new(ActionMode::Wall, *pos, axis, dir);
        }
        // Type Double
        if self.is_free(&(pos + dir + dir), stat) &&
           self.is_free(&(pos + dir - up), stat) &&
           self.is_free(&(pos + dir + dir - up), stat) {
            //println!("Type B");
            return Action::new(ActionMode::Double, *pos, axis, dir);
        }
        Action::none(*pos)
    }
}
impl PartialEq for Grid {
    fn eq(&self, other: &Grid) -> bool {
        self.robots == other.robots
    }
}
impl Eq for Grid {}

impl std::hash::Hash for Grid {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.robots.hash(state);
    }
}
impl PartialOrd for Grid {
    fn partial_cmp(&self, other: &Grid) -> Option<std::cmp::Ordering> {
        self.robots.partial_cmp(&other.robots)
    }
}
impl Ord for Grid {
    fn cmp(&self, other: &Grid) -> std::cmp::Ordering {
        self.robots.cmp(&other.robots)
    }
}

impl std::fmt::Debug for Grid {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{}", self)
    }
}
impl std::fmt::Display for Grid {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        let (min, max) = self.bounds();
        let min = min.min_bound(&Vector::default());
        let max = max.max_bound(&Vector::default());
        
        for _ in min.x..=max.x {
            write!(f, "==")?;
        }
        write!(f, "\n")?;
        for z in (min.z)..=max.z {
            if z > min.z {
                for _ in min.x..=max.x+1 {
                    write!(f, "**")?;
                }
                write!(f, "\n")?;
            }
            for y in (min.y..=max.y).rev() {
                for x in min.x..=max.x {
                    print_bot(self, Vector::new(x,y,z), f)?;
                }
                write!(f, "\n")?;
            }
        }
        for _ in (min.x)..(max.x+1) {
            write!(f, "==")?;
        }
        Ok(())
    }
}
fn print_bot(grid: &Grid, pos: Vector, f: &mut std::fmt::Formatter) -> std::fmt::Result {
    const TEXTURES: [&str; 8] = ["  ", "..", "--", "==", "[]", "()", "{}", "{}"];
    let mut i = 0;
    if grid.robots.contains(&pos) {
        i += 4;
    }
    if grid.reserved.contains_key(&pos) {
        i += 2;
    }
    if pos == Vector::default() {
        i += 1;
    }
    write!(f, "{}", TEXTURES[i])
}

// Function for converting a grid state into a tex document, using a custom \drawcube macro
#[allow(unused)]
pub fn print_tikz(grid: &Grid, name: &str) {
    use std::fs::File;
    use std::io::Write;

    let (min, max) = grid.bounds();

    println!("{} {}", min, max);

    let mut f = File::create(format!("tikz/{}.tex", name)).unwrap();
    
    f.write_all(format!("  \\drawgrid{{{}}}{{{}}}{{{}}}{{{}}}\n", min.x, -max.z, max.x+1, -min.z+1).as_bytes()).unwrap();

    for r in grid.iter() {
        let color = if r.manhattan_size() % 2 == 0 {"gray"}
                    else {"gray!80"};
        f.write_all(format!("    \\drawcube{{{}}}{{{}}}{{{}}}{{{}}}{{}}{{0}}\n", r.x, -r.z, r.y, color).as_bytes()).unwrap();
    }
}