//! A* algorithm for planning cubot movement using the 'hungarian' heuristic.
//! 
//! see [astar_hungarian_search].

use crate::{environment::*, generators};
use std::{collections::{BinaryHeap, HashMap}, sync::{Arc, atomic::{AtomicBool, Ordering}}};
use im::OrdSet;

#[derive(Debug, Clone)]
struct Node {
    grid: Grid,
    robots: OrdSet<Vector>,
    g: usize,
    f: usize
}
impl Ord for Node {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        other.f.cmp(&self.f).then(other.g.cmp(&self.g))
    }
}
impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        self.f == other.f && self.g == other.g
    }
}
impl Eq for Node {}

/// Astar algorithm for solving the cubot planning problem.
/// Gets two [Grid] instances and plans a path between them, with allowed actions being one [Action] of one robot.
/// The heuristic function finds a pairing of goals and cubots with the minimal sum, using the [crate::heuristics::hungarian] algorithm.
/// Input includes a [Statistic] struct and an boolean used for prematurely stopping the runtime.
pub fn astar_hungarian_search(start: Grid, goal: Grid, stat: &mut Statistic, cancel: Arc<AtomicBool>) -> Option<Plan> {
    let mut counter = 0;

    let mut opened = BinaryHeap::<Node>::new();
    let mut seen = HashMap::<Grid, usize>::new();
    let mut previous = HashMap::<Grid, Action>::new();
    let robots = start.clone().robots;
    
    let heuristic = crate::heuristics::hungarian_size;

    let mut s = Statistic::new();
    let h = heuristic(&start, &goal);

    seen.insert(start.clone(), 0);
    opened.push(Node {
        grid: start.clone(),
        robots: robots.clone(),
        g: 0,
        f: h
    });

    while let Some(node) = opened.pop() {

        // Every 1024 iterations check if the algorithm should stop
        if counter % 1024 == 0 && cancel.load(Ordering::Relaxed) {
            return None;
        }
        //println!("{}\nG: {} F: {}", node.grid, node.g, node.f);

        s.max_frontier_size(opened.len());

        if node.grid == goal {
            let mut path = Plan::new();
            let mut grid = node.grid;

            while let Some(&act) = previous.get(&grid) {
                path.push(act);
                grid.perform_unchecked(act.inverse(), stat);
            }
            stat.max_states_seen(seen.len());
            stat.add_states_processed(seen.len() - opened.len());

            path.reverse();
            return Some(path);
        }
        let moveable: Vec<_> = node.robots
                .iter()
                .filter(|b| crate::algorithms::removeable(&node.grid, b, stat))
                .collect();

        let actions: Vec<_> = moveable.into_iter()
                .map(|b| node.grid.possible_actions(b, stat))
                .flatten()
                .collect();

        for act in actions {
            let mut new_grid = node.grid.clone();
            new_grid.perform_unchecked(act, stat);
            let new_g = node.g + 1;

            stat.add_grid_comparisons(1);
            if let Some(&g) = seen.get(&new_grid) &&
                   new_g >= g {
                continue;
            }
            stat.add_grid_comparisons(1);
            seen.insert(new_grid.clone(), new_g);
            previous.insert(new_grid.clone(), act);

            let h = heuristic(&new_grid, &goal);
            let new_f = new_g + h;

            opened.push(Node {
                grid: new_grid.clone(),
                robots: node.robots.without(&act.start()).update(act.end()),
                g: new_g,
                f: new_f
            });
        }
        //crate::utils::wait_for_enter();
    }
    stat.max_states_seen(seen.len());
    stat.add_states_processed(seen.len() - opened.len());

    return None;
}
/// Helper function, gets one grid and dissasembles it into canonical form
pub fn astar_hungarian_canon(grid: Grid, stat: &mut Statistic, cancel: Arc<AtomicBool>) -> Option<Plan> {
    let n = grid.count();
    let goal = generators::canonical(0, 0, n, &mut 0);

    astar_hungarian_search(grid, goal, stat, cancel)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_astar() {
        let results = vec![0, 1, 4, 11, 20, 31, 46];

        let mut start = Grid::new();
        let mut goal = Grid::new();
        let mut stat = Statistic::new();
        let mut dummy_stat = Statistic::new();
        let s = 3;
        for i in 0..=s {
            start.insert(Vector::new(i,0,0), &mut dummy_stat);
            start.insert(Vector::new(0,i,0), &mut dummy_stat);
            goal.insert(Vector::new(i,0,0), &mut dummy_stat);
            goal.insert(Vector::new(s,i,0), &mut dummy_stat);
        }
        let cancel = Arc::new(AtomicBool::new(false));

        let res = astar_hungarian_search(start.clone(), goal, &mut stat, cancel);
        let res = res.unwrap();

        crate::utils::animate_plan(start, &res, 500);

        assert_eq!(res.len(), results[s as usize]);
        println!("{}", stat);
    }
}