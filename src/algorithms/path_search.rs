//! Backtracking Path Planning Search implementation
//! 
//! see [backtrack_path_search].

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
    d_steps: usize
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
        (other.p_steps).cmp(&(self.p_steps))).then(
        (other.d_steps).cmp(&(self.d_steps)))
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

/// Gets a [Grid] and returns a plan to dissasemble it into the canonical structure.
/// Input includes a [Statistic] struct and an Atomic boolean used for prematurely stopping the runtime.
/// 
/// The algorithm works by dissasembling and reassembling the structure, using an intermediate canonical structure.
/// The top layer is an A* algorithm that has two types of actions availiable:
/// - D (dijstra) steps - single action of a cubot
/// - P (path) steps - pre-planned path of a cubot into the canonical structure. Using it removes the cubot from the pool of availiable cubots.
/// The algorithm prioritizes states with the most removed cubots. If equal, it prioritizes states with fewer D steps performed.
/// The algorithm isn't optimal but it is complete, finding a solution if there is one.
pub fn backtrack_path_search(start: Grid, stat: &mut Statistic, cancel: Arc<AtomicBool>) -> Option<Plan> {
    let mut opened = BinaryHeap::<Node>::new();
    let mut seen = BTreeMap::<Grid, Datum>::new();
    let mut opened_count = 0;

    let first = Node {
        grid: start.clone(),
        goal: -1,
        d_steps: 0,
        p_steps: 0
    };
    seen.insert(start.clone(), Datum::default());
    opened.push(first);

    let mut counter = 0;
    while let Some(node) = opened.pop() {
        stat.max_frontier_size(opened.len());

        

        opened_count += 1;

        //println!("{}", node.grid);
        //println!("{}\nGoal: {} P: {} D: {}", grid, goal, node.p_steps, node.d_steps);

        if -node.goal as usize == start.count() {
            let mut total_path = Plan::new();
            let mut grid = node.grid;

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
        
        // Dijkstra steps
        let movable: Vec<_> = node.grid  
                .iter()
                .filter(|b| b.x > 0 || b.y != 0 || b.z != 0)
                .filter(|b| crate::algorithms::removeable(&node.grid, b, stat))
                .collect();
        let actions: Vec<_> = movable.into_iter()
                .map(|b| node.grid.possible_actions(b, stat))
                .flatten()
                .collect();

        for act in actions {
            let mut new_grid = node.grid.clone();
            new_grid.perform_unchecked(act, stat);
            let new_d = node.d_steps + 1;
            let cost = node.p_steps + new_d;

            stat.add_grid_comparisons(1);
            if let Some(previous) = seen.get(&new_grid) &&
                   cost >= previous.cost {
                continue;
            }
            stat.add_grid_comparisons(1);
            seen.insert(new_grid.clone(), (cost, act.start(), act.end()).into());

            opened.push(Node {
                grid: new_grid.clone(),
                goal: node.goal,
                d_steps: new_d,
                p_steps: node.p_steps
            });
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
            let cost = node.d_steps + new_p;

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
                d_steps: node.d_steps
            });
        }

        //wait_for_enter();
    }
    stat.max_states_seen(seen.len());
    stat.add_states_processed(seen.len() - opened.len());

    return None;
}
mod tests {
    use crate::{utils::*};
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
        let plan = backtrack_path_search(start.clone(), &mut stat, Arc::new(AtomicBool::new(false)));
        assert!(plan.is_some());
        let plan = plan.unwrap();
        //assert_eq!(path.len(), 19);

        println!("{}", s);
        animate_plan(start, &plan, 100);
    }
    #[test]
    fn test_overhang() {
        let mut start = Grid::new();
        let mut stat = Statistic::new();
        let mut dummy_stat = Statistic::new();

        let s = 4;
        for i in 0..s {
            start.insert(Vector::new(0,i,0), &mut dummy_stat);
            start.insert(Vector::new(s-1,i,0), &mut dummy_stat);
            start.insert(Vector::new(i,s-1,0), &mut dummy_stat);
        }
        start.insert(Vector::new(0,s,0), &mut dummy_stat);

        println!("{}", start);
        let plan = backtrack_path_search(start.clone(), &mut stat, Arc::new(AtomicBool::new(false)));
        assert!(plan.is_some());
        let plan= plan.unwrap();
        //assert_eq!(path.len(), 19);

        println!("{}", s);
        animate_plan(start, &plan, 100);
    }
    #[test]
    fn test_l() {
        let mut start = Grid::new();
        let mut end = Grid::new();
        let mut stat = Statistic::new();
        let mut dummy_stat = Statistic::new();
        let s = 10;
        for i in 0..=s {
            start.insert(Vector::new(i,0,0), &mut dummy_stat);
            start.insert(Vector::new(0,i,0), &mut dummy_stat);
            end.insert(Vector::new(i,0,0), &mut dummy_stat);
            end.insert(Vector::new(s,i,0), &mut dummy_stat);
        }
        println!("{}", start);

        let h = crate::heuristics::closest_heuristic;
        let ser1 = backtrack_path_search(start.clone(), &mut stat, Arc::new(AtomicBool::new(false))).unwrap();
        let ser2 = backtrack_path_search(end.clone(), &mut stat, Arc::new(AtomicBool::new(false))).unwrap();
        let ser = ser1
            .into_iter()
            .chain(ser2.into_iter()
                .map(|a| a.inverse())
                .rev())
            .collect();

        crate::utils::animate_plan(start, &ser, 100);
        print!("{}", stat);
        println!("  Length: {}", ser.len());
    }
}
