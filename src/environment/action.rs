//! Struct for representing a cubot's action in a grid.
//! 
//! An action is uniquely defined with its position, axis of rotation, initial direction of rotation, and size. 
//! The struct, instead of storing the size, stores the 5 possible types of actions:
//! - Flat, Wall, Jump - all rotations by 90 degrees. Functionally they are identical but are separated to allow for a more constrained move set.
//! - Double - rotation by 180 degrees.
//! - None - rotation by 0 degrees, i.e. a non-move. The Action's position should still be valid and point to an existing cubot, but the axis and direction are arbitrary.

use crate::environment::*;

/// Type of action, represented by an enum
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum ActionMode {
    None, Flat, Wall, Jump, Double
}
use ActionMode::*;

/// Struct uniquelly representing a cubot's action in a grid.
/// 
/// # Example
/// 
/// ```
/// let s = Vector::new(1, 2, 3);
/// let a = Action::new(ActionMode::Flat, s, UnitVector::Pz, UnitVector::Px);
/// let b = a.inverse();
/// let v = b.end();
/// assert_eq!(v, s);
/// ```
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct Action {
    pub mode: ActionMode,
    pub position: Vector,
    pub axis: UnitVector,
    pub direction: UnitVector
}
impl Action {
    /// Shortcut for checking if an action moves the cubot.
    pub fn is_none(&self) -> bool {
        self.mode == None
    }
    /// Shortcut for creating a non-moving action.
    pub fn none(position: Vector) -> Self { Self {
        mode: None,
        position,
        axis: UnitVector::None,
        direction: UnitVector::None
    }}
    /// Creates a new Action struct.
    pub fn new(mode: ActionMode, position: Vector, axis: UnitVector, direction: UnitVector) -> Self { Self {
        mode, position, axis, direction
    }}
    /// Returns the starting position of an action.
    pub fn start(&self) -> Vector {
        self.position
    }
    /// Calculates and returns the ending position of an action.
    pub fn end(&self) -> Vector {
        match self.mode {
            None => self.position,
            Flat | 
            Wall | 
            Jump => self.position +
                    self.direction,
            Double => self.position +
                      self.direction +
                      self.direction.cross(self.axis)
        }
    }
    /// Creates an action that complements this one.
    /// If applied after each other, the cubot will end in the same position it began.
    pub fn inverse(&self) -> Action {
        if self.mode == None {
            return *self;
        }
        // Wall and Jump actions are complements of each other, the remaining actions are their own complements. 
        let mode = match self.mode {
            Wall => Jump,
            Jump => Wall,
            _ => self.mode
        };
        // For Double actions, the move direction not pointing to the end position, so it cannot be simply inverted, and has to be calculated differently.
        let direction = if self.mode == Double {
            self.axis.cross(self.direction)
        } else {
            -self.direction
        };
        Action { mode, position: self.end(), axis: -self.axis, direction }
    }
    /// Returns the list of all positions the cubot will pass through while moving. Includes the starting and ending positions
    pub fn reserved_positions(&self) -> Vec<Vector> {
        match self.mode {
            None => vec![],
            Flat | 
            Wall | 
            Jump => {
                let up = self.axis.cross(self.direction);
                vec![
                self.position,
                self.position + up,
                self.position + up + self.direction,
                self.position + self.direction
            ]},
            Double => { 
                let up = self.axis.cross(self.direction);
                vec![
                self.position,
                self.position + up,
                self.position + up + self.direction,
                self.position + self.direction,
                self.position + self.direction + self.direction,
                self.position + self.direction + self.direction - up,
                self.position + self.direction - up,
            ]}
        }
    }
    /// Returns the list of all positions the cubot will need to remain solid during the action
    pub fn solid_positions(&self) -> Vec<Vector> {
        match self.mode {
            None => vec![],
            Flat => vec![
                self.position + self.direction.cross(self.axis),
                self.position + self.direction + self.direction.cross(self.axis)
            ],
            Wall => vec![
                self.position + self.direction.cross(self.axis),
                self.position + self.direction + self.direction
            ],
            Jump => vec![
                self.position - self.direction,
                self.position + self.direction + self.direction.cross(self.axis)
            ],
            Double => vec![
                self.position + self.direction.cross(self.axis)
            ]
        }
    }
}
impl std::fmt::Display for Action {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        if self.mode == None {
            write!(f, "None({})", self.position)
        } else {
            write!(f, "{:?}({} {} {})", self.mode, self.position, self.axis, self.direction)
        }
    }
}

impl std::hash::Hash for Action {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.position.hash(state);
    }
}