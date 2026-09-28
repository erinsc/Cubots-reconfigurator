//! 3D integer vector used to represent the position in a grid.
//!
//! There are two vector structs defined:
//! 
//! - [`Vector`] - The typical integer vector, used for positions and offsets
//! - [`UnitVector`] - Lighter version of a vector that only allows vectors of length 0 or 1, used for offsets only.

use std::ops::{Neg, Add, AddAssign, Sub, SubAssign};
use std::cmp::Ordering;

/// 3d integer vector.
/// Implements basic vector operations as needed. Most arithmetic is defined between Vector and [UnitVector], not between Vectors directly.
/// Uses lexicographical ordering `(y, x, z)` because of the intended use in search algorithms.
/// 
/// # Example
/// 
/// ```
/// let a = Vector::new(1, 2, 3);
/// let b = Vector::unit(1);
/// let c = a - b;
/// assert_equal!(c, Vector::new(0, 1, 2));
/// ```
#[derive(Debug, Default, Copy, Clone, PartialEq, Eq, Hash)]
pub struct Vector {
    pub x: i16,
    pub y: i16,
    pub z: i16
}
impl Vector {
    /// Creates a new 3D vector.
    pub fn new(x: i16, y: i16, z: i16) -> Self { Self {
        x, y, z
    }}
    /// Creates a zero vector.
    pub fn default() -> Self {
        Self::new(0, 0, 0)
    }
    /// Creates a *unit* vector, with a given value in all coordinates.
    pub fn unit(i: i16) -> Self {
        Self::new(i, i, i)
    }
    /// Returns the manhattan distance from the center.
    pub fn manhattan_size(&self) -> u16 {
        self.x.unsigned_abs() + self.y.unsigned_abs() + self.z.unsigned_abs()
    }
    /// Returns the largest coordinate.
    /// Used for finding the minimum bounding cuboid of a set of vectors.
    pub fn max_bound(&self, other: &Self) -> Self { Self {
        x: self.x.max(other.x),
        y: self.y.max(other.y),
        z: self.z.max(other.z),
    }}
    /// Returns the smallest coordinate.
    /// Used for finding the minimum bounding cuboid of a set of vectors.
    pub fn min_bound(&self, other: &Self) -> Self { Self {
        x: self.x.min(other.x),
        y: self.y.min(other.y),
        z: self.z.min(other.z),
    }}
}

impl std::fmt::Display for Vector {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "[{}, {}, {}]", self.x, self.y, self.z)?;
        Ok(())
    }
}

impl Ord for Vector {
    /// Lexigograpic ordering for the vectors.
    /// The ordering is `(y, x, z)`, since the intended use was for dissasembly algorithms on a flat plane.
    /// Positions higher up are considered less desireable. This hasn't been used directly however.
    fn cmp(&self, other: &Self) -> Ordering {
        self.y.cmp(&other.y).then(self.x.cmp(&other.x)).then(self.z.cmp(&other.z))
    }
}
impl PartialOrd for Vector {
    fn partial_cmp(&self, other: &Vector) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Lighter version of the Vector struct that only allows vectors of length 0 or 1.
/// Implements basic vector operations as needed.
/// 
/// # Example
/// 
/// ```
/// let v = Vector::new(1, 0, 0);
/// let u = UnitVector::Px;
/// let r = v + u;
/// assert_eq!(r, Vector(2, 0, 0));
/// ```
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum UnitVector {
    None, Px, Nx, Py, Ny, Pz, Nz
}
use UnitVector::*;
impl UnitVector {
    /// Rotation around `(1, 1, 1)` by 120 degrees.
    /// Shortcut for getting a vector that's perpendicular to another.
    pub fn permute(self) -> Self {
        match self {
            Px => Py, Py => Pz, Pz => Px,
            Nx => Ny, Ny => Nz, Nz => Nx,
            None => None
        }
    }
    /// Cross product.
    /// Defined with a LUT instead of multiplication, makes it faster.
    pub fn cross(self, other: Self) -> Self {
        match (self, other) {
            (Px, Py) => Pz, (Py, Px) => Nz,
            (Px, Ny) => Nz, (Ny, Px) => Pz,
            (Nx, Py) => Nz, (Py, Nx) => Pz,
            (Nx, Ny) => Pz, (Ny, Nx) => Nz,

            (Px, Pz) => Ny, (Pz, Px) => Py,
            (Px, Nz) => Py, (Nz, Px) => Ny,
            (Nx, Pz) => Py, (Pz, Nx) => Ny,
            (Nx, Nz) => Ny, (Nz, Nx) => Py,

            (Py, Pz) => Px, (Pz, Py) => Nx,
            (Py, Nz) => Nx, (Nz, Py) => Px,
            (Ny, Pz) => Nx, (Pz, Ny) => Px,
            (Ny, Nz) => Px, (Nz, Ny) => Nx,

            _ => None
        }
    }
}
impl std::fmt::Display for UnitVector {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{:?}", self)?;
        Ok(())
    }
}

impl Neg for UnitVector {
    type Output = Self;
    fn neg(self) -> Self {
        match self {
            Px => Nx, Py => Ny, Pz => Nz,
            Nx => Px, Ny => Py, Nz => Pz,
            None => None
        }
    }
}
impl AddAssign<UnitVector> for Vector {
    fn add_assign(&mut self, dir: UnitVector) {
        match dir {
            Px => self.x += 1,
            Py => self.y += 1,
            Pz => self.z += 1,
            Nx => self.x -= 1,
            Ny => self.y -= 1,
            Nz => self.z -= 1,
            None => {}
        }
    }
}
impl Add<UnitVector> for Vector {
    type Output = Self;

    fn add(self, dir: UnitVector) -> Vector {
        let mut copy = self;
        copy += dir;
        copy
    }
}
impl Add<UnitVector> for &Vector {
    type Output = Vector;

    fn add(self, dir: UnitVector) -> Vector {
        let mut copy = *self;
        copy += dir;
        copy
    }
}
impl SubAssign<UnitVector> for Vector {
    fn sub_assign(&mut self, dir: UnitVector) {
        match dir {
            Px => self.x -= 1,
            Py => self.y -= 1,
            Pz => self.z -= 1,
            Nx => self.x += 1,
            Ny => self.y += 1,
            Nz => self.z += 1,
            None => {}
        }
    }
}
impl Sub<UnitVector> for Vector {
    type Output = Self;

    fn sub(self, dir: UnitVector) -> Vector {
        let mut copy = self;
        copy -= dir;
        copy
    }
}
impl Sub<UnitVector> for &Vector {
    type Output = Vector;

    fn sub(self, dir: UnitVector) -> Vector {
        let mut copy = *self;
        copy -= dir;
        copy
    }
}
impl SubAssign<&Vector> for Vector {
    fn sub_assign(&mut self, other: &Vector) {
        self.x -= other.x;
        self.y -= other.y;
        self.z -= other.z;
    }
}
impl Sub<&Vector> for &Vector {
    type Output = Vector;

    fn sub(self, dir: &Vector) -> Vector {
        let mut copy = *self;
        copy -= dir;
        copy
    }
}