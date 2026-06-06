# Digital Traffic Management System

A C++ data structures project that models a small Islamabad road network and applies graph algorithms to traffic management tasks such as shortest routing, traffic-aware travel estimation, signal timing, and network analysis.

## Project Idea

The system treats intersections as graph vertices and roads as weighted directed edges.

- Distance data is loaded from `Islamabad_Intersections_Distance_Int.csv`
- Traffic intensity data is loaded from `Islamabad_Traffic_Intensity.csv`
- Blocked roads are represented by `999` in the CSV files

## DSA Concepts Used

- Adjacency matrix for direct edge lookup
- Adjacency list for efficient graph traversal
- Hash map (`unordered_map`) for fast intersection lookup by name
- Priority queue based Dijkstra for shortest and fastest path queries
- Floyd-Warshall for all-pairs shortest path analysis
- Path reconstruction using parent arrays

## Features

1. Find VIP route using shortest distance
2. Inspect traffic on a direct road segment
3. Generate green signal timings from traffic thresholds
4. Find fastest route using a traffic-aware weighted graph
5. Show network-wide graph analytics

## Files

- `Final_Project.cpp` - main source code
- `Islamabad_Intersections_Distance_Int.csv` - distance adjacency matrix
- `Islamabad_Traffic_Intensity.csv` - traffic intensity adjacency matrix
- `Report.docx` - project report

## How It Works

The program loads two graphs from CSV files:

- A distance graph
- A traffic intensity graph

It then builds a third derived graph called the travel-time graph. For every road:

`travel_time = distance * minutes_per_kilometer_based_on_traffic`

This makes the fastest-route feature algorithmically correct, because Dijkstra runs on travel-time edge weights instead of mixing separate shortest-distance and shortest-traffic paths.

## Build

If you have MSYS2 `g++` installed:

```powershell
& "C:\msys64\ucrt64\bin\g++.exe" -std=c++17 -Wall -Wextra -pedantic -static -static-libstdc++ -static-libgcc "Final_Project.cpp" -o "final_project.exe"
```

## Run

```powershell
.\final_project.exe
```

## Sample Checks

- Option `1` then `G-7 Markaz` and `F-10 Markaz`
- Option `4` then `G-7 Markaz` and `F-10 Markaz`
- Option `5` for network summary
- Option `2` then `F-8 Markaz` and `F-10 Markaz` to see a blocked direct road case

## Algorithm Notes

- Dijkstra on the distance graph returns the shortest route by kilometers
- Dijkstra on the travel-time graph returns the fastest route by estimated minutes
- Floyd-Warshall is used to compute graph center, diameter, and reachability statistics
