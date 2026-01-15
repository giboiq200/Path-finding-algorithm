# Pathfinding & Optimization Toolkit

A comprehensive implementation and visualization of fundamental pathfinding algorithms with rigorous performance analysis and complexity optimization. Built in C++ with raylib for real-time visualization.

![C++](https://img.shields.io/badge/C%2B%2B-14-blue.svg)
![raylib](https://img.shields.io/badge/raylib-5.0-red.svg)
![Algorithms](https://img.shields.io/badge/algorithms-4-green.svg)

## 🎯 Focus Areas

**This project demonstrates:**
- Algorithmic correctness and implementation accuracy
- Time and space complexity analysis
- Performance optimization techniques
- Graph traversal and search strategies
- Real-world problem-solving with pathfinding
- Debugging complex algorithmic systems

## 📊 Implemented Algorithms

### Core Pathfinding Algorithms

| Algorithm | Type | Optimality | Complexity | Use Case |
|-----------|------|-----------|-----------|----------|
| **A*** | Informed Search | Optimal* | O((V+E) log V) | Best general-purpose pathfinding |
| **Dijkstra** | Uniform Cost | Optimal | O((V+E) log V) | Shortest path in weighted graphs |
| **BFS** | Uninformed Search | Optimal** | O(V + E) | Unweighted shortest path |
| **DFS** | Uninformed Search | Non-optimal | O(V + E) | Graph exploration |

\* Optimal with admissible heuristic  
\*\* Optimal for unweighted graphs

### Heuristic Functions

| Heuristic | Formula | Characteristics | Best For |
|-----------|---------|-----------------|----------|
| **Manhattan** | `|x₁-x₂| + |y₁-y₂|` | 4-directional movement | Grid-based, no diagonals |
| **Euclidean** | `√((x₁-x₂)² + (y₁-y₂)²)` | True distance | Any-direction movement |
| **Diagonal (Chebyshev)** | `max(|x₁-x₂|, |y₁-y₂|)` | 8-directional movement | Grid with diagonals |

## 🏗️ Architecture

### Design Principles

**Algorithmic Correctness**
- Each algorithm implements the canonical version from academic literature
- Rigorous testing against known pathfinding scenarios
- Edge case handling (unreachable goals, blocked paths)

**Performance Optimization**
- Priority queue implementations using `std::priority_queue`
- Hash-based visited set lookups (O(1) average)
- Early termination on goal discovery
- Memory-efficient data structures

**Clean Abstractions**
- Polymorphic algorithm interface
- Pluggable heuristic functions
- Separation of algorithm logic from visualization
- Generic grid representation

### Data Structures

```cpp
// Priority Queue (A*, Dijkstra)
std::priority_queue<Node, std::vector<Node>, std::greater<Node>>

// Hash Set (Visited tracking)
std::unordered_set<Vec2i>

// Hash Map (Path reconstruction, cost tracking)
std::unordered_map<Vec2i, Vec2i>
std::unordered_map<Vec2i, float>
```

## 🔬 Complexity Analysis

### Time Complexity

| Algorithm | Best Case | Average Case | Worst Case |
|-----------|-----------|--------------|------------|
| A* | O(bd) | O(b^(d/2)) | O(b^d) |
| Dijkstra | O((V+E) log V) | O((V+E) log V) | O((V+E) log V) |
| BFS | O(V + E) | O(V + E) | O(V + E) |
| DFS | O(V + E) | O(V + E) | O(V + E) |

Where:
- `V` = number of vertices (grid cells)
- `E` = number of edges (connections)
- `b` = branching factor
- `d` = depth of solution

### Space Complexity

| Algorithm | Space Complexity | Notes |
|-----------|------------------|-------|
| A* | O(b^d) | Stores all frontier nodes |
| Dijkstra | O(V) | Stores distance for all nodes |
| BFS | O(V) | Queue size bounded by vertices |
| DFS | O(h) | Stack depth (h = height) |

## 🎮 Controls

### Algorithm Selection
- **1** - A* Algorithm
- **2** - Dijkstra's Algorithm
- **3** - Breadth-First Search (BFS)
- **4** - Depth-First Search (DFS)

### Execution
- **SPACE** - Run selected algorithm
- **A** - Run all algorithms (performance comparison)
- **C** - Clear visualization (keep walls)
- **R** - Reset grid (regenerate maze)

### Editing
- **W** - Wall mode (draw/erase walls)
- **S** - Start mode (set start position)
- **E** - End mode (set end position)
- **H** - Cycle heuristics (A* only)

### Mouse
- **Left Click** - Place current edit mode
- **Right Click** - Erase walls

## 📈 Performance Comparison

The toolkit provides real-time performance metrics:

```
=== PERFORMANCE COMPARISON ===
A*:
  Time: 0.4521 ms
  Nodes: 234
  Path: 45 steps
  Cost: 45.0000

Dijkstra:
  Time: 0.8934 ms
  Nodes: 512
  Path: 45 steps
  Cost: 45.0000

BFS:
  Time: 0.7823 ms
  Nodes: 498
  Path: 45 steps
  Cost: 45.0000

DFS:
  Time: 0.3421 ms
  Nodes: 789
  Path: 156 steps
  Cost: 156.0000
```

### Key Insights

**A* vs Dijkstra:**
- A* explores ~50% fewer nodes due to heuristic guidance
- A* is faster despite slightly more complex per-node operations
- Both guarantee optimal paths

**BFS vs DFS:**
- BFS guarantees shortest path, DFS does not
- DFS uses less memory but explores more nodes
- BFS is better for shortest-path problems

**Heuristic Impact:**
- Manhattan: Fastest for grid-based movement
- Euclidean: More accurate for diagonal movement
- Better heuristic = fewer nodes explored

## 🧪 Testing Scenarios

### Optimal Path Testing
1. Create clear path between start and end
2. Run A* and Dijkstra
3. Verify both find same shortest path
4. Compare node exploration counts

### Heuristic Effectiveness
1. Run A* with each heuristic
2. Compare nodes explored
3. Verify all find optimal path
4. Analyze time differences

### Worst-Case Scenarios
1. Create complex maze with many dead ends
2. Place goal far from start
3. Run all algorithms
4. Compare performance degradation

### Edge Cases
1. **Unreachable Goal**: Wall off the end point
2. **No Path**: Surround start completely
3. **Trivial Path**: Start adjacent to goal
4. **Single Path**: Create narrow corridor

## 📚 Algorithm Details

### A* Algorithm

**Pseudocode:**
```
function A*(start, goal):
    openSet = {start}
    cameFrom = {}
    
    gScore[start] = 0
    fScore[start] = heuristic(start, goal)
    
    while openSet is not empty:
        current = node in openSet with lowest fScore
        
        if current == goal:
            return reconstructPath(cameFrom, current)
        
        openSet.remove(current)
        closedSet.add(current)
        
        for neighbor in neighbors(current):
            if neighbor in closedSet:
                continue
            
            tentative_g = gScore[current] + distance(current, neighbor)
            
            if tentative_g < gScore[neighbor]:
                cameFrom[neighbor] = current
                gScore[neighbor] = tentative_g
                fScore[neighbor] = gScore[neighbor] + heuristic(neighbor, goal)
                
                if neighbor not in openSet:
                    openSet.add(neighbor)
    
    return failure
```

**Key Properties:**
- **Completeness**: Always finds a solution if one exists
- **Optimality**: Finds optimal path with admissible heuristic
- **Efficiency**: Explores fewer nodes than uninformed search

**Heuristic Admissibility:**
A heuristic h(n) is admissible if:
```
h(n) ≤ actual_cost(n, goal) for all n
```

Our heuristics are admissible because:
- Manhattan never overestimates (cannot move diagonally)
- Euclidean is the true straight-line distance
- Diagonal (Chebyshev) is exact for 8-directional movement

### Dijkstra's Algorithm

**Key Insight**: Dijkstra is A* with h(n) = 0

**Properties:**
- Guaranteed optimal path
- Explores nodes in order of distance from start
- No heuristic needed (works on general graphs)
- More node exploration than A*

### BFS vs DFS

**BFS (Breadth-First):**
- Explores level by level
- Guarantees shortest path (unweighted)
- Uses queue (FIFO)
- Higher memory usage

**DFS (Depth-First):**
- Explores one branch completely
- Does not guarantee shortest path
- Uses stack (LIFO)
- Lower memory usage

## 🚀 Advanced Topics

### Optimization Techniques

**1. Early Termination**
```cpp
if (current.pos == end) {
    return result; // Stop immediately
}
```

**2. Closed Set Optimization**
```cpp
if (closedSet.find(neighbor) != closedSet.end()) {
    continue; // Skip already-explored nodes
}
```

**3. Priority Queue Efficiency**
```cpp
std::priority_queue<Node, std::vector<Node>, std::greater<Node>>
// O(log n) insertion and extraction
```

### Heuristic Design

**Properties of Good Heuristics:**
1. **Admissible**: Never overestimate
2. **Consistent**: h(n) ≤ cost(n, n') + h(n')
3. **Informative**: Close to actual cost
4. **Fast to compute**: O(1) ideally

**Trade-offs:**
- More accurate heuristic = fewer nodes explored
- Complex heuristic = slower per-node computation
- Sweet spot: accurate yet fast calculation

## 🎓 Educational Value

This project demonstrates understanding of:

- **Graph Theory**: Traversal algorithms, shortest paths
- **Data Structures**: Priority queues, hash maps, sets
- **Algorithm Analysis**: Time/space complexity, Big-O notation
- **Optimization**: Heuristic design, early termination
- **Debugging**: Complex state tracking, edge cases
- **Software Engineering**: Clean abstractions, modularity

## 📊 Benchmarking

To run comprehensive benchmarks:

```cpp
// Press 'A' in application for automated comparison
// Or check console output for detailed statistics
```

**Metrics Collected:**
- Execution time (milliseconds)
- Nodes explored (visited count)
- Path length (steps)
- Path cost (total weight)
- Success rate (found/not found)

## 🐛 Debugging Features

- **Visual Feedback**: See algorithm exploration in real-time
- **Step-by-step**: Observe node visitation order
- **Path Reconstruction**: Verify correctness visually
- **Performance Metrics**: Identify bottlenecks
- **Edge Case Testing**: Easily test corner cases


## 🤝 Contributing

Contributions welcome! Areas for improvement:
- Additional algorithms (IDA*, Jump Point Search)
- More heuristics (Octile distance)
- 3D pathfinding support
- Weighted terrain
- Dynamic obstacle avoidance
