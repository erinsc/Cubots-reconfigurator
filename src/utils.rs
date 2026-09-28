//! Helper functions used for debugging and visualizing algorithms

use std::io::{self, Write};
use crate::{environment::*, trial::Trial};
use std::fs::File;


/// Helper function, when ran simply pauses the program and waits for the user to read whatever logs they need to.
/// Pressing enter resumes the program.
pub fn wait_for_enter() {
    print!("Press Enter to continue...");
    io::stdout().flush().unwrap();
    let mut input = String::new();
    io::stdin().read_line(&mut input).unwrap();
    println!("Continuing...");
}
/// Animates the given plan of cubot actions in the console.
/// Delay between frames is given as input.
#[allow(unused)]
pub fn animate_plan(mut grid: Grid, actions: &Plan, delay: u64) {
    wait_for_enter();
    println!("{}", grid);
    let mut dummy_stat = Statistic::new();

    for a in actions {
        grid.queue_unchecked(*a, &mut dummy_stat);

        println!("{} {}", grid, a);
        std::thread::sleep(std::time::Duration::from_millis(delay));

        grid.execute_queue(&mut dummy_stat);
    }
    println!("{}", grid);
}
/// Animates the given parallel plan of cubot actions in the console.
/// Delay between frames is given as input.
#[allow(unused)]
pub fn animate_parallelplan(mut grid: Grid, actions: &ParallelPlan, delay: u64) {
    wait_for_enter();
    println!("{}", grid);
    let mut dummy_stat = Statistic::new();

    for t in actions {
        for a in t.iter() {
            grid.queue_unchecked(*a, &mut dummy_stat);
        }
        println!("{}", grid);
        std::thread::sleep(std::time::Duration::from_millis(delay));
        grid.execute_queue(&mut dummy_stat);
    }
    println!("{}", grid);
}
/// Stores a vector of trials into a textfile.
#[allow(unused)]
pub fn record(trials: Vec<Trial>, name: &str) {
    use std::io::Write;

    let mut f = File::create(format!("results/{}.txt", name)).unwrap();
    for trial in trials {
        f.write_all(format!("{}", trial).as_bytes()).unwrap();
    }
}
/// Prints out a [Vec] of [Trial]s to the console.
pub fn print_trials(trials: Vec<Trial>, name: &str) {
    println!("\n\nRECORDING {}", name);
    for trial in trials {
        println!("{}", trial);
    }
}
