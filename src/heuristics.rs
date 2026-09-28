//! Collection of heuristics used by path-planning algorithms. Not all of them are currently used.

#![allow(unused)]
use crate::environment::*;

/// Heuristic for a single cubot. returns the minimum number of actions required. It's a tight heuristic, meaning it doesn't underestimate either.
pub fn min_steps(from: &Vector, to: &Vector) -> usize {
    let d = from - to;
    (d.x.abs() as usize + d.z.abs() as usize).max(d.y.abs() as usize)
}
/// Returns how many cubots aren't in the right position between two structures.
pub fn matching(froms: &Grid, tos: &Grid) -> usize {
    let froms = froms.clone();
    let tos = tos.clone();
    froms.robots.difference(tos.robots).len()
}
pub fn closest_from_set(from: &Vector, tos: &Grid) -> usize {
    tos.iter()
       .map(|to| min_steps(from, to))
       .min()
       .unwrap_or(0)
}
/// Returns heuristic distance from a canonical structure.
pub fn from_canon(grid: &Grid) -> usize {
    let r = grid.iter()
                .map(|p| (p.x.abs() + p.z.abs() + p.y.max(0)) as usize)
                .sum();
    r
}
/// Heuristic as sum of coordinates.
pub fn coordinate_sum(froms: &Grid, tos: &Grid) -> usize {
    from_canon(froms).abs_diff(from_canon(tos))
}

/// Each cubot finds its closest goal, distances are summed as the heuristic.
pub fn closest_heuristic(froms: &Grid, tos: &Grid) -> usize {
    let r = froms.iter()
                 .map(|from| closest_from_set(from, tos))
                 .sum();
    r
}
/// Sum of pairings with minimal sum of distances
pub fn hungarian_size(froms: &Grid, tos: &Grid) -> usize {
    hungarian(froms, tos).1
}
/// Returns the sum of the smallest pairing between goals and starts.
pub fn hungarian(froms: &Grid, tos: &Grid) -> (Vec<usize>, usize) {
    let n = froms.count();
    
    let froms: Vec<_> = froms.iter().collect();
    let tos: Vec<_> = tos.iter().collect();

    let table: Vec<Vec<usize>> = (0..n)
        .map(|i| (0..n)
            .map(|j| min_steps(froms[i], tos[j]))
            .collect()
        )
        .collect();

    let r = hungarian_alg(table);

    r
}
/// Implementation of the hungarian matching algorithm, taken from Wikipedia
pub fn hungarian_alg(c: Vec<Vec<usize>>) -> (Vec<usize>, usize) {
    let n = c.len();

    let mut job = vec![usize::MAX; n+1];
    let mut ys = vec![0; n];
    let mut yt = vec![0; n+1];
    let mut answer = 0;

    for j_cur in 0..n {
        let mut w_cur = n;
        job[w_cur] = j_cur;

        let mut min_to = vec![usize::MAX; n+1];
        let mut prev = vec![usize::MAX; n+1];
        let mut in_z = vec![false; n+1];

        while job[w_cur] != usize::MAX {
            in_z[w_cur] = true;
            let j = job[w_cur];
            let mut delta = usize::MAX;
            let mut w_next = 3435;

            for w in 0..n {
                if !in_z[w] {
                    let pd = c[j][w] + yt[w] - ys[j];
                    if pd < min_to[w] {
                        min_to[w] = pd;
                        prev[w] = w_cur;
                    }
                    if min_to[w] < delta {
                        delta = min_to[w];
                        w_next = w;
                    }
                }
            }

            for w in 0..=n {
                if in_z[w] {
                    ys[job[w]] += delta;
                    yt[w] += delta;
                } else {
                    min_to[w] -= delta;
                }
            }
            w_cur = w_next;
        }
        let mut w;
        while w_cur != n {
            w = prev[w_cur];
            job[w_cur] = job[w];
            w_cur = w;
        }
        answer = yt[n];
    }
    job.pop();

    (job, answer)
}