//! A helper function to check whether a cubot is removeable from the grid while keeping the rest of the structure connected.
//! 
//! see [removeable].

use crate::environment::*;
use im::OrdSet;
use std::collections::{BinaryHeap, HashMap};

#[derive(Debug, Copy, Clone)]
struct Node {
    pos: Vector,
    g: usize,
    f: usize
}
impl Ord for Node {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        other.f.cmp(&self.f)
    }
}
impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        self.f == other.f
    }
}
impl Eq for Node {}

fn heuristic(from: &Vector, to: &Vector) -> usize {
    (from - to).manhattan_size() as usize
}
// Returns all neighbours of a cubot.
fn neighbors(grid: &Grid, pos: &Vector, stat: &mut Statistic) -> impl Iterator<Item = Vector> {
    use UnitVector::*;
    [Px, Nx, Py, Ny, Pz, Nz].iter()
            .map(move |&d| pos + d)
            .filter(|p| grid.is_stable(p, stat))
}

/// Function for checking whether a cubot can be removed.
/// 
/// The algorithm notes down all neighbours of the to-be-removed cubot, and tries to find a path connecting them all.
/// If there are 0 or 1 neighbours present the algorithm automatically succeeds.
/// Otherwise a simple A* is used to explore the connected cubots.
/// Input includes a [Statistic] struct.
pub fn removeable(grid: &Grid, robot: &Vector, stat: &mut Statistic) -> bool {
    if !grid.is_stable(robot, stat) {
        return false;
    }
    let mut goals: OrdSet<Vector> = neighbors(grid, robot, stat).collect();
    if goals.len() < 2 {
        return true;
    }
    // A* search
    let mut opened = BinaryHeap::<Node>::new();
    let mut seen = HashMap::<Vector, usize>::new();

    let first = goals.iter().next().unwrap();
    seen.insert(*robot, 0);
    seen.insert(*first, 0);
    opened.push(Node {
        pos: *first,
        g: 0,
        f: heuristic(&first, robot)
    });

    while let Some(node) = opened.pop() {
        stat.max_frontier_size(opened.len());
        goals.remove(&node.pos);

        if goals.len() == 0 {
            stat.max_states_seen(seen.len());
            stat.add_states_processed(seen.len() - opened.len());
            return true;
        }
        for new_pos in neighbors(grid, &node.pos, stat) {
            let new_g = node.g + 1;

            if let Some(previous) = seen.get(&new_pos) &&
                   new_g >= *previous {
                continue;
            }
            seen.insert(new_pos, new_g);

            opened.push(Node {
                pos: new_pos,
                g: new_g,
                f: new_g + heuristic(&new_pos, robot)
            });
        }
    }
    stat.max_states_seen(seen.len());
    stat.add_states_processed(seen.len() - opened.len());
    return false;
}
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_removeable() {
        let mut stat = Statistic::new();
        let mut dummy_stat = Statistic::new();
        let mut grid = Grid::new();
        grid.insert(Vector::new(0,0,0), &mut dummy_stat);
        grid.insert(Vector::new(0,1,0), &mut dummy_stat);
        grid.insert(Vector::new(0,2,0), &mut dummy_stat);

        assert!(removeable(&grid, &Vector::new(0,0,0), &mut stat)); // Cant move but can be removed
        assert!(!removeable(&grid, &Vector::new(0,1,0), &mut stat)); // Cant remove without splitting
        assert!(removeable(&grid, &Vector::new(0,2,0), &mut stat)); // Can remove
        assert!(!removeable(&grid, &Vector::new(0,3,0), &mut stat)); // Not occupied
    }
}