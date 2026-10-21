# Smart Route & Delivery Optimizer 2.0

A classroom-ready DAA mini software. The UI is HTML/CSS/JavaScript, the lightweight local API is Python, and the actual algorithm engine is C++17.

## Run on macOS
1. Extract the ZIP.
2. Right-click `run.command` → Open (the first time macOS may ask for confirmation).
3. The script compiles `optimizer.cpp`, starts the local server, and opens http://localhost:8080.

Requirements: Apple Command Line Tools / a C++17 compiler and Python 3.

## Features
- Add/edit delivery points, package weight and priority.
- 1–4 delivery vehicles with configurable capacity.
- Light / medium / heavy traffic simulation.
- Manually block roads and recalculate.
- Nearest Neighbor greedy tour.
- 2-Opt local-search improvement.
- Dijkstra shortest-path analysis.
- A* shortest-path analysis using Euclidean heuristic.
- Algorithm distance, execution time and complexity table.
- Capacity-aware vehicle assignment and individual routes.
- Estimated trip cost and graphical route visualization.

## DAA explanation
The delivery locations are vertices in a weighted graph. Nearby points form simulated road edges. Edge weights are based on Euclidean distance and a traffic multiplier. A blocked road removes that edge.

**Nearest Neighbor:** greedily chooses the nearest unvisited delivery point. Approximate time complexity O(n²).

**2-Opt:** repeatedly reverses route segments when that reduces the tour cost. A pass examines O(n²) swaps.

**Dijkstra:** computes a shortest path on non-negative weighted road edges. With a priority queue, O((V+E) log V).

**A\*:** uses straight-line distance as an admissible-style geometric heuristic for guided shortest-path search in this simulated map. The UI also shows expanded nodes so you can discuss search efficiency.

**Multi-vehicle allocation:** deliveries are considered by priority and assigned subject to vehicle capacity, then each vehicle route is optimized.

## Files
- `optimizer.cpp` — C++ algorithms and JSON result generator
- `server.py` — local HTTP bridge between browser and C++
- `index.html` — dashboard UI
- `run.command` — one-click macOS launcher

## Suggested classroom demo
1. Run the default dataset and show the NN → 2-Opt saving.
2. Change traffic from Medium to Heavy and optimize again.
3. Click **Block Road**, enter two visible point indexes, then optimize.
4. Change the number/capacity of vehicles and explain the fleet split.
5. Use the Algorithm Analysis table to discuss time complexity and shortest-path vs full-tour optimization.

Note: This is an educational optimizer using synthetic coordinates/roads, not a live GPS navigation service.
# Smart-Route-Delivery-Optimizer
