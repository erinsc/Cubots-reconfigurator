//! A* algorithm for planning the path of a single cubot through an otherwise static environment

use crate::environment::*;
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

struct Datum {
    g: usize,
    from: Action
}
impl Datum {
    fn default() -> Datum { Datum {
        g: 0,
        from: Action::none(Vector::default())
    }}
}
impl From<(usize, Action)> for Datum {
    fn from(tuple: (usize, Action)) -> Datum { Datum {
        g: tuple.0,
        from: tuple.1
    }}
}

/// Finds the shortest path between two given [Vector] positions and a [Grid] to plan through.
pub fn find_path(grid: &Grid, start: Vector, goal: Vector, stat: &mut Statistic) -> Option<Plan> {
    let mut opened = BinaryHeap::<Node>::new();
    let mut seen = HashMap::<Vector, Datum>::new();

    let heuristic = crate::heuristics::min_steps;

    seen.insert(start, Datum::default());
    opened.push(Node {
        pos: start,
        g: 0,
        f: heuristic(&start, &goal)
    });

    while let Some(node) = opened.pop() {
        stat.max_frontier_size(opened.len());

        if goal == node.pos {
            let mut path = Vec::new();
            let mut current = node.pos;
            while let Some(mov) = seen.get(&current) && !mov.from.is_none() {
                path.push(mov.from);
                current = mov.from.start();
            }
            path.reverse();

            stat.max_states_seen(seen.len());
            stat.add_states_processed(seen.len() - opened.len());

            return Some(path);
        }

        for action in grid.possible_actions(&node.pos, stat) {
            let new_pos = action.end();
            let new_g = node.g + 1;

            if let Some(previous) = seen.get(&new_pos) &&
                   new_g >= previous.g {
                continue;
            }
            seen.insert(new_pos, (new_g, action).into());

            opened.push(Node {
                pos: new_pos,
                g: new_g,
                f: new_g + heuristic(&new_pos, &goal)
            });
        }
    }
    stat.max_states_seen(seen.len());
    stat.add_states_processed(seen.len() - opened.len());

    return None;
}
#[allow(unused)]
mod tests {
    use super::*;

    #[test]
    fn test_pathsearch() {
        let mut start = Grid::new();
        let mut stat = Statistic::new();
        let mut dummy_stat = Statistic::new();
        let s = 10;
        for i in 0..=s {
            start.insert(Vector::new(s,i,0), &mut dummy_stat);
            start.insert(Vector::new(i,0,0), &mut dummy_stat);
        }
        let path = find_path(&start, Vector::new(s,s+1,0), Vector::new(-1,0,0), &mut stat);
        assert!(path.is_some());
        let path = path.unwrap();
        assert_eq!(path.len(), 20);
    }
}