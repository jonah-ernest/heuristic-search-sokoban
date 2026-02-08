# Heuristic Search for Sokoban Planning

This project implements heuristic search algorithms for solving Sokoban planning problems. The focus is on designing informed heuristics and anytime search strategies that efficiently navigate large combinatorial state spaces while avoiding deadlock configurations.

The system integrates custom heuristics into weighted and iterative search procedures to improve solution quality under limited computation time.

---

## Project Overview

Sokoban is a classic planning problem in artificial intelligence involving robots pushing boxes to designated storage locations in a grid environment with obstacles.

In this project I:

- Designed deadlock-aware heuristics that improve on Manhattan-distance baselines
- Implemented greedy box-to-storage and robot-to-box assignment strategies
- Added corner and cluster deadlock detection
- Implemented Anytime Weighted A*
- Implemented Iterative A*
- Implemented Anytime Greedy Best-First Search (GBFS)
- Evaluated algorithm behavior across benchmark problem instances

The result is a modular planning system capable of producing increasingly optimal solutions as runtime increases.

---

## Repository Structure

- `solution.py` — Custom heuristics and search algorithms
- `search.py` — Generic search framework
- `sokoban.py` — Sokoban environment representation

---

## Methods Implemented

### Heuristics
- Manhattan distance baseline
- Deadlock-aware heuristics
- Greedy matching for box-to-storage assignments
- Robot-to-box distance estimation

### Search Algorithms
- Weighted A*
- Iterative A*
- Anytime Greedy Best-First Search

---

## Tools & Technologies

- Python
- Heuristic search
- State-space planning
- Anytime algorithms
- Graph traversal
- Optimization under time constraints
