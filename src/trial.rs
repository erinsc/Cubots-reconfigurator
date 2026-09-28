//! Struct representing multiple runs of an algorithm.
//! 
//! A trial consists of N runs of an algorithm on deterministically generated structures. 
//! Multiple statistics are recorded and calculated to allow evaluating efficiency of an algorithm.

use std::{
    io::{Write, stdout}, 
    sync::{Arc, atomic::{AtomicBool, Ordering}}, 
    time::{Duration, Instant},
    usize
};
use tokio::time::timeout as join_timeout;
use crate::environment::*;

/// Struct for storing a single metric for N runs.
/// Stores the minimum, maximum, and average values recorded.
#[derive(Debug, Clone)]
pub struct Metric {
    pub min: Option<usize>,
    pub max: Option<usize>,
    pub total: usize,
    pub count: usize
}
impl Metric {
    fn new() -> Self { Self {
        min: None,
        max: None,
        total: 0,
        count: 0
    }}
    fn avg(&self) -> Option<f64> {
        if self.count != 0 {
            Some(self.total as f64 / self.count as f64)
        } else {
            None
        }
    }
    fn record(&mut self, new: usize) {
        self.min = Some(self.min.map_or(new, |old| old.min(new)));
        self.max = Some(self.max.map_or(new, |old| old.max(new)));
        self.total += new;
        self.count += 1;
    }
}

/// Struct storing many different statistics about N runs of an algorithm.
#[derive(Debug, Clone)]
pub struct Trial {
    pub agents: usize, // Number of agents
    pub success_rate: f64, // Percentage of suns that succeeded
    pub timeout_rate: f64, // Percentage of runs that timed out
    pub time: Metric,    
    pub path_length: Metric,
    pub states_seen: Metric,
    pub states_processed: Metric,
    pub frontier_size: Metric,
    pub actions_tested: Metric,
    pub grid_actions: Metric,
    pub grid_comparisons: Metric
}
impl Trial {
    fn new() -> Trial { Trial {
        agents: 0,
        success_rate: 0.0,
        timeout_rate: 0.0,
        time: Metric::new(),
        path_length: Metric::new(),
        states_seen: Metric::new(),
        states_processed: Metric::new(),
        frontier_size: Metric::new(),
        actions_tested: Metric::new(),
        grid_actions: Metric::new(),
        grid_comparisons: Metric::new()
    }}
}

/// Function for running an algorithm multiple times on deterministically generated structures and recording the results.
/// The function re-runs the algorithm with different numbers of agents, and records a trial for each one.
/// Function inputs include number of runs to perform, and number of seconds the algorithm is given to succeed.
pub async fn measure<G, A> (
    algorithm: A,
    generator: G,
    agent_counts: &Vec<usize>,
    timeout: u64,
    run_count: usize
) -> Vec<Trial>
where    
    G: Fn(usize, &mut usize) -> Grid + Send,
    A: Fn(Grid, &mut Statistic, Arc<AtomicBool>) -> Option<Plan> + Send + 'static + Copy,
{
    let mut stdout = stdout();

    let mut trials = vec![];
    println!("Starting measurement");

    let mut seed = 42;

    for ac in agent_counts {
        println!("\n{} Agent trial: ", ac);
        let mut successes = 0;
        let mut timeouts = 0;
        
        let mut trial = Trial::new();

        for run in 0..run_count {
            print!("\rRun {:2}/{}...", run+1, run_count);
            stdout.flush().unwrap();

            let mut stat = Statistic::new();
            let start = generator(*ac, &mut seed);

            let t0 = Instant::now();
            let cancel = Arc::new(AtomicBool::new(false));
            let cancopy = cancel.clone();

            let alg = move || {
                //println!("S");
                let plan = algorithm(start, &mut stat, cancopy);
                //println!("E");
                (plan, stat)
            };
            let task = tokio::task::spawn_blocking(alg);

            // Possible results of the algorithm:
            // 1. The algorithm succeeds, returns the Statistic struct
            // 2. Timeout, the algorithm didnt finish in time
            // 3. Failure, the algorithm didnt find a solution
            match join_timeout(Duration::from_secs(timeout), task).await {
                Ok(Ok((plan, stat))) => {
                    if let Some(plan) = plan {
                        print!("Success  ");
                        trial.path_length.record(plan.len());
                        successes += 1;
                    }
                    else {
                        print!("No path");
                    }
                    trial.states_seen.record(stat.states_seen);
                    trial.states_processed.record(stat.states_processed);
                    trial.frontier_size.record(stat.frontier_size);
                    trial.actions_tested.record(stat.actions_tested);
                    trial.grid_actions.record(stat.grid_actions);
                    trial.grid_comparisons.record(stat.grid_comparisons);
                },
                Ok(Err(_)) |
                   Err(_) => {
                    print!("Timed out");
                    cancel.store(true, Ordering::Relaxed);
                    timeouts += 1;
                },
            };
            let t = t0.elapsed().as_secs() as usize;
            trial.time.record(t);
        }
        trial.agents = *ac;
        trial.success_rate = successes as f64 / run_count as f64;
        trial.timeout_rate = timeouts as f64 / run_count as f64;
        trials.push(trial);
        //println!("{}", trial);

        if timeouts >= run_count.div_ceil(2) {
            println!("Ending trials early, the algorithm reached its limit");
            break;
        }
    }
    trials
}
impl std::fmt::Display for Metric {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        if self.count > 0 {
            write!(f, "Min: {:9} | Max: {:9} | Avg: {:13.3}", self.min.unwrap_or(1), self.max.unwrap_or(2), self.avg().unwrap_or(3.0))
        }
        else {
            write!(f, "No Trials Recorded")
        }
    }
}

impl std::fmt::Display for Trial {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "\nTrial results with {} agents, {} runs\n", self.agents, self.time.count)?;
        write!(f, "------------------------------------------------------------------------\n")?;
        write!(f, "  Time (seconds):   {}\n", self.time)?;
        write!(f, "  Path length:      {}\n", self.path_length)?;
        write!(f, "  States seen:      {}\n", self.states_seen)?;
        write!(f, "  States processed: {}\n", self.states_processed)?;
        write!(f, "  Frontier size:    {}\n", self.frontier_size)?;
        write!(f, "  Actions tested:   {}\n", self.actions_tested)?;
        write!(f, "  Grid actions:     {}\n", self.grid_actions)?;
        write!(f, "  Grid comparisons: {}\n", self.grid_comparisons)?;
        write!(f, "      Success rate: {:7.3}%\n", self.success_rate*100.0)?;
        write!(f, "      Timeout rate: {:7.3}%\n", self.timeout_rate*100.0)?;
        Ok(())
    }
}
