# Krychloboti simulátor

Simple rust project for simulating the cubot planning environment and testing the performance of different algorithms.

## Cubots

A cubot is a robot the shape of a cube, which can connect to other peers and form a larger structure. It can pivot around the edge of another cubot when certain requirements are met, and change the structure.

The cubot planning problem gets two different structures with the same number of cubots, and asks for a sequence of actions that changes one structure into the other. The goal is for the algorithm to finish quickly, and to return short solutions. 

## Tested algorithms

Because of the high cooperation of cubots required, typical multi-agent algorithms like CBS aren't viable. This project is a part of a thesis intended to find a new efficient algorithm, BPPS.

The algorithms tested in this work:

- A* - Typical A* implementation, assuming the structure is one agent. The movement of any cubot counts as one action. Two different heuristics are tested.
- greedy search - Modified version of A* which ignores the cost metric, only using the heuristic.
- Prioritized Planning - An algorithm that searches for a permutation of cubots, in which the cubots can plan individual paths outside the structure. One all cubots leave, they return in the reverse order to construct the target structure.
- BPPS - like Prioritized Planning, dissasembles and reasembles the structure. Preplans paths of individual robots and uses them as actions, aswell as individual actions. Heuristic prioritizes states with smallest numbers of remaining robots, and minimal actions taken.

## Documentation

you can see the programmer documentation here: https://erinsc.github.io/Cubots-reconfigurator/

## How to run

As this wasn’t intended for public viewing, the project does not have a way to be run through the console. To run different algorithms or with different settings, the `main` function is to be adjusted and the project recompiled. The documentation includes an example of what the `main` function should typically look like.

Modify the `main` function appropriatelly, then recompile and run the project using `cargo run`. 
