//! Collection of generators for cubot structures with different properties.

#![allow(unused)]

use crate::environment::*;

fn randrange(seed: &mut usize, max: i16) -> i16 {
    const A: usize = 1664525;
    const C: usize = 1013904223;

    *seed = seed.wrapping_mul(A).wrapping_add(C);
    ((*seed >> 32) % max as usize) as i16
}
/// Returns a canonical structure with the given number of cubots.
pub fn canonical(_: i16, _: i16, total: usize, _: &mut usize) -> Grid {
    let mut grid = Grid::new();
    let mut dummy_stat = Statistic::new();
    for x in 0..total as i16 {
        grid.insert(Vector::new(-x, 0, 0), &mut dummy_stat);
    }
    grid
}
/// Returns a structure of randomly generated columns of heights 1 to max_z, with the given number of columns.
pub fn columns(max_y: i16, max_z: i16, total: usize, seed: &mut usize) -> Grid {
    let mut grid = Grid::new();
    let mut dummy_stat = Statistic::new();
    let (mut x, mut z) = (0, 0);
    let mut total = total;

    while total > 0 {
        let r = randrange(seed, max_y);
        let height = ((r * r * r / (max_y * max_y)) as usize)
            .max(1).min(total);
        total -= height;
        for y in 0..height as i16 {
            grid.insert(Vector::new(x, y, z), &mut dummy_stat);
        }
        x += (z+1) / max_z;
        z = (z+1) % max_z;
    }
    grid
}
/// Same as [columns], makes sure the width, depth, and height/2 are approximatelly the same.
pub fn columns_equivalent(total: usize, seed: &mut usize) -> Grid {
    let max_y = (total as f64).powf(1.0/3.0) as i16;
    let max_z = max_y;
    
    let mut grid = Grid::new();
    let mut dummy_stat = Statistic::new();
    let (mut x, mut z) = (0, 0);
    let mut total = total;

    while total > 0 {
        let r = randrange(seed, max_y*2) + 1;
        let height = (r as usize)
            .max(1).min(total);
        total -= height;
        for y in 0..height as i16 {
            grid.insert(Vector::new(x, y, z), &mut dummy_stat);
        }
        x += (z+1) / max_z;
        z = (z+1) % max_z;
    }
    grid
}
/// Generates a columns structure with some random overhangs.
pub fn overhangs(max_y: i16, max_z: i16, total: usize, seed: &mut usize) -> Grid {
    let mut grid = Grid::new();
    let mut dummy_stat = Statistic::new();
    let (x, mut z) = (0, 0);
    let mut total = total;

    while total > 0 {
        let r = randrange(seed, max_y);
        let height = ((r * r * r / (max_y * max_y)) as usize)
            .max(1).min(total) as i16;
        for y in 0..height {
            if grid.insert(Vector::new(x, y, z), &mut dummy_stat).is_none() {
                total -= 1;
            }
        }
        use UnitVector::*;
        let d = [Px, Nx, Pz, Nz][randrange(seed, 3) as usize];
        let r = randrange(seed, 3);
        let mut pos = Vector::new(x, height-1, z);
        for _ in 0..r {
            pos += d;
            if grid.insert(pos, &mut dummy_stat).is_none() {
                total -= 1;
            }
        }
        z = (z+1) / max_z;
    }
    grid
}
/// Generates a "bridge" structure, with a given number of cubots.
pub fn bridge(max_y: i16, _: i16, mut total: usize, _seed: &mut usize) -> Grid {
    let mut grid = Grid::new();
    let mut dummy_stat = Statistic::new();

    total -= 4;
    let d = (total / 3) as i16;

    let max_y = d + {total % 3 == 2} as i16;
    let max_x = d + {total % 3 == 1} as i16;

    println!("{} {}", max_x, max_y);

    for i in 0..max_y {
        grid.insert(Vector::new(0, i, 0), &mut dummy_stat);
        grid.insert(Vector::new(max_x-1, i, 0), &mut dummy_stat);
    }
    for i in 0..max_x {
        grid.insert(Vector::new(i, max_y-1, 0), &mut dummy_stat);
    }

    grid
}
/// Same as [bridge], makes sure width and height are approximatelly the same.
pub fn bridge_equivalent(mut total: usize, _seed: &mut usize) -> Grid {
    let mut grid = Grid::new();
    let mut dummy_stat = Statistic::new();

    total;
    let d = (total / 3) as i16;

    let h = d + {total % 3 == 2} as i16;
    let w = d + {total % 3 == 1} as i16;

    println!("{}, {}", w, h);

    for i in 0..h {
        grid.insert(Vector::new(0, i, 0), &mut dummy_stat);
        grid.insert(Vector::new(w-1, i, 0), &mut dummy_stat);
    }
    for i in 0..w {
        grid.insert(Vector::new(i, h-1, 0), &mut dummy_stat);
    }
    grid.insert(Vector::new(0, h, 0), &mut dummy_stat);
    grid.insert(Vector::new(0, h+1, 0), &mut dummy_stat);

    grid
}
/// Generates a structure using a random 3d walk.
pub fn random_walk(max_y: i16, max_z: i16, total: usize, seed: &mut usize) -> Grid {
    let mut grid = Grid::new();
    let mut dummy_stat = Statistic::new();
    let mut pos = Vector::default();
    let mut total = total;

    while total > 0 {
        if grid.insert(pos, &mut dummy_stat).is_none() {
            total = total.saturating_sub(1);
        }
        use UnitVector::*;
        let d = [Px, Nx, Py, Ny, Pz, Nz][randrange(seed, 6) as usize];
        let new_pos = pos + d + d;
        if new_pos.x < 0 || new_pos.y < 0 || new_pos.z < 0
            || new_pos.y >= max_y || new_pos.z >= max_z {
            continue;
        }
        if grid.insert(pos+d, &mut dummy_stat).is_none() {
            total = total.saturating_sub(1);
        }
        pos = new_pos;
    }
    grid
}
/// Same as [random_walk], does not have a width constraint.
pub fn random_walk_equivalent(total: usize, seed: &mut usize) -> Grid {
    random_walk(total as i16, total as i16, total, seed)
}
/// Returns a specific hostile structure that cannot be disassembled.
pub fn knot() -> Grid {
    let mut start = Grid::new();
    let mut dummy_stat = Statistic::new();

    start.insert(Vector::new(1,0,0), &mut dummy_stat);
    start.insert(Vector::new(0,0,0), &mut dummy_stat);
    start.insert(Vector::new(0,1,0), &mut dummy_stat);

    start.insert(Vector::new(0,2,0), &mut dummy_stat);
    start.insert(Vector::new(0,2,1), &mut dummy_stat);
    start.insert(Vector::new(1,2,1), &mut dummy_stat);
    start.insert(Vector::new(2,2,1), &mut dummy_stat);
    start.insert(Vector::new(2,2,0), &mut dummy_stat);
    start.insert(Vector::new(2,2,-1), &mut dummy_stat);
    start.insert(Vector::new(1,2,-1), &mut dummy_stat);

    start.insert(Vector::new(1,2,-2), &mut dummy_stat);
    start.insert(Vector::new(1,3,-2), &mut dummy_stat);
    start.insert(Vector::new(1,4,-2), &mut dummy_stat);
    start.insert(Vector::new(1,4,-1), &mut dummy_stat);
    start.insert(Vector::new(1,4,0), &mut dummy_stat);

    start.insert(Vector::new(1,3,0), &mut dummy_stat);

    start
}
/// Generates a structure in a tree-like fashion, growing up and to the sides until the desired number of cubots is reached.
pub fn strongly_connected(mut total: usize, seed: &mut usize) -> Grid {
    let mut grid = Grid::new();
    let mut dummy_stat = Statistic::new();

    use UnitVector::*;
    let mut dirs = |pos| [Px, Nx, Pz, Nz, Py].iter().map(|&d| (pos, d)).collect::<Vec<_>>();
    let mut shuffle = |vec: &mut Vec<(Vector, UnitVector)>| {
        for i in (1..vec.len()).rev() {
            let j = randrange(seed, (i + 1) as i16) as usize;
            vec.swap(i, j);
        }
    };

    let root = Vector::default();
    let mut frontier = dirs(root);

    grid.insert(root, &mut dummy_stat);
    total -= 1;

    shuffle(&mut frontier);
    while total > 0 && let Some((pos, dir)) = frontier.pop() {
        let (l, r) = (dir.cross(Py), dir.cross(Ny));
        let next = pos + dir;

        if next.x < 0 || next.y < 0 || next.z < 0 {
            continue;
        }
        
        if grid.contains(&next, &mut dummy_stat) ||
           grid.contains(&(next + l), &mut dummy_stat) ||
           grid.contains(&(next + r), &mut dummy_stat) {
            continue;
        }
        grid.insert(next, &mut dummy_stat);
        total -= 1;
        frontier.append(&mut dirs(next));
        if dir != Py {
            frontier.push((next, dir));
            frontier.push((next, dir));
        };
        shuffle(&mut frontier);
    }

    grid
}
