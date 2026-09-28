//! Priority Planning Search
//! 
//! see [prioritized_planning_search].

use std::collections::{BinaryHeap, HashMap, BTreeMap};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use crate::environment::*;
use crate::algorithms::find_path;

#[derive(Clone, PartialEq, Eq, Debug)]
struct Node {
    grid: Grid,
    goal: i16, // is negative
    p_steps: usize,
}
/*impl Ord for Node {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        other.goal.cmp(&self.goal).then(
        (other.p_steps + other.d_steps).cmp(&(self.p_steps + self.d_steps)))
    }
}*/

impl Ord for Node {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        other.goal.cmp(&self.goal).then(
        (other.p_steps).cmp(&(self.p_steps)))
    }
}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

#[derive(Debug, Clone, Default)]
struct Datum {
    cost: usize,
    from: Vector,
    to: Vector
}

impl From<(usize, Vector, Vector)> for Datum {
    fn from(tuple: (usize, Vector, Vector)) -> Datum { Datum {
        cost: tuple.0,
        from: tuple.1,
        to: tuple.2
    }}
}

/// Prioritized Planning search.
/// 
/// Gets a starting [Grid] and dissasembles it into a canonical form.
/// Input includes a [Statistic] struct and an Atomic boolean used for prematurely stopping the runtime.
/// The algorithm works by dissasembling and reassembling the structure, using an intermediate canonical structure.
/// Each cubot plans its path individually, assuming the others are static.
/// an A* algorithm searches through possible orderings of cubots, finishing when it finds one that allows all cubots to reach the canonical structure.
/// The algorithm isn't complete, and it is possible for it to fail even when a solution exists. It is however much faster than typical A* algorithms.

pub fn prioritized_planning_search(start: Grid, stat: &mut Statistic, cancel: Arc<AtomicBool>) -> Option<Plan> {
    let mut opened = BinaryHeap::<Node>::new();
    let mut seen = BTreeMap::<Grid, Datum>::new();
    let mut opened_count = 0;

    let first = Node {
        grid: start.clone(),
        goal: -1,
        p_steps: 0
    };
    seen.insert(start.clone(), Datum::default());
    opened.push(first);

    while let Some(node) = opened.pop() {
        stat.max_frontier_size(opened.len());

        opened_count += 1;

        //println!("{}", node.grid);
        //println!("{}\nGoal: {} P: {} D: {}", grid, goal, node.p_steps, node.d_steps);

        let mut counter = 0;

        if -node.goal as usize == start.count() {
            let mut total_path = Plan::new();
            let mut grid = node.grid;

            if counter % 1024 == 0 && cancel.load(Ordering::Relaxed) {
                return None;
            }

            while let Some(previous) = seen.remove(&grid) && previous.from != previous.to {
                stat.add_grid_comparisons(1);
                
                grid.remove(&previous.to, stat);
                let path = find_path(&grid, previous.from, previous.to, stat);
                grid.insert(previous.from, stat);

                total_path.append(&mut path.unwrap());
            }
            total_path.reverse();

            stat.max_states_seen(seen.len());
            stat.add_states_processed(opened_count);

            return Some(total_path);
        }
        // Path steps
        let robots: Vec<_> = node.grid
                .iter()
                .filter(|b| b.x > 0 || b.y != 0 || b.z != 0 )
                .filter(|b| crate::algorithms::removeable(&node.grid, b, stat))
                .collect();

        for robot in robots {
            let mut new_grid = node.grid.clone();
            new_grid.remove(robot, stat);
            let goal = Vector::new(node.goal, 0, 0);

            let result = find_path(&new_grid, *robot, goal, stat);

            if result.is_none() {
                continue;
            }
            new_grid = node.grid.clone();
            let new_p = node.p_steps + result.unwrap().len();
            let cost = new_p;

            new_grid.remove(robot, stat);
            new_grid.insert(goal, stat);

            stat.add_grid_comparisons(1);
            if let Some(previous) = seen.get(&new_grid) &&
                   cost >= previous.cost {
                continue;
            }
            stat.add_grid_comparisons(1);
            seen.insert(new_grid.clone(), (cost, *robot, goal).into());

            opened.push(Node {
                grid: new_grid.clone(),
                goal: node.goal -1,
                p_steps: new_p,
            });
        }

        //wait_for_enter();
    }
    stat.max_states_seen(seen.len());
    stat.add_states_processed(seen.len() - opened.len());

    return None;
}
mod tests {
    use crate::{generators, utils::*};
    use super::*;

    #[test]
    fn test_plannable() {
        let mut start = Grid::new();
        let mut stat = Statistic::new();
        let mut dummy_stat = Statistic::new();
        let s = 3;
        for i in 0..(s*s) {
            start.insert(Vector::new(i%s,i/s,0), &mut dummy_stat);
        }
        let plan = prioritized_planning_search(start.clone(), &mut stat, Arc::new(AtomicBool::new(false)));
        assert!(plan.is_some());
        let plan = plan.unwrap();
        //assert_eq!(path.len(), 19);

        println!("{}", s);
        animate_plan(start, &plan, 100);
    }
    #[test]
    fn test_columns() {
        let mut start = crate::generators::columns(10, 4, 20, &mut 40);
        let mut stat = Statistic::new();
        let mut dummy_stat = Statistic::new();
        println!("{}", start);
        let plan = prioritized_planning_search(start.clone(), &mut stat, Arc::new(AtomicBool::new(false)));
        assert!(plan.is_some());
        let plan= plan.unwrap();
        //assert_eq!(path.len(), 19);

        animate_plan(start, &plan, 100);
    }
}
