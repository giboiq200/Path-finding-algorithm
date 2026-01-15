// =============================================================================
// PATHFINDING & OPTIMIZATION TOOLKIT
// Algorithms: A*, Dijkstra, BFS, DFS with performance analysis
// =============================================================================

#include <raylib.h>
#include <vector>
#include <queue>
#include <stack>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <string>
#include <sstream>
#include <iomanip>

// =============================================================================
// CORE DATA STRUCTURES
// =============================================================================

struct Vec2i {
    int x, y;

    Vec2i() : x(0), y(0) {}
    Vec2i(int x, int y) : x(x), y(y) {}

    bool operator==(const Vec2i& o) const { return x == o.x && y == o.y; }
    bool operator!=(const Vec2i& o) const { return !(*this == o); }

    Vec2i operator+(const Vec2i& o) const { return Vec2i(x + o.x, y + o.y); }
};

// Hash function for Vec2i to use in unordered_map/set
namespace std {
    template<>
    struct hash<Vec2i> {
        size_t operator()(const Vec2i& v) const {
            return hash<int>()(v.x) ^ (hash<int>()(v.y) << 1);
        }
    };
}

// =============================================================================
// HEURISTICS
// =============================================================================

class Heuristic {
public:
    virtual float calculate(const Vec2i& a, const Vec2i& b) const = 0;
    virtual const char* getName() const = 0;
    virtual ~Heuristic() {}
};

class ManhattanHeuristic : public Heuristic {
public:
    float calculate(const Vec2i& a, const Vec2i& b) const override {
        return (float)(std::abs(a.x - b.x) + std::abs(a.y - b.y));
    }
    const char* getName() const override { return "Manhattan"; }
};

class EuclideanHeuristic : public Heuristic {
public:
    float calculate(const Vec2i& a, const Vec2i& b) const override {
        int dx = a.x - b.x;
        int dy = a.y - b.y;
        return std::sqrt((float)(dx * dx + dy * dy));
    }
    const char* getName() const override { return "Euclidean"; }
};

class DiagonalHeuristic : public Heuristic {
public:
    float calculate(const Vec2i& a, const Vec2i& b) const override {
        int dx = std::abs(a.x - b.x);
        int dy = std::abs(a.y - b.y);
        return (float)(std::max(dx, dy));
    }
    const char* getName() const override { return "Diagonal (Chebyshev)"; }
};

// =============================================================================
// GRID REPRESENTATION
// =============================================================================

enum CellType {
    EMPTY = 0,
    WALL = 1,
    START = 2,
    END = 3,
    PATH = 4,
    VISITED = 5,
    FRONTIER = 6
};

class Grid {
    int width, height;
    std::vector<CellType> cells;
    Vec2i start, end;

public:
    Grid(int w, int h) : width(w), height(h), start(1, 1), end(w - 2, h - 2) {
        cells.resize(width * height, EMPTY);

        // Create borders
        for (int x = 0; x < width; x++) {
            set(x, 0, WALL);
            set(x, height - 1, WALL);
        }
        for (int y = 0; y < height; y++) {
            set(0, y, WALL);
            set(width - 1, y, WALL);
        }
    }

    int getWidth() const { return width; }
    int getHeight() const { return height; }

    CellType get(int x, int y) const {
        if (x < 0 || x >= width || y < 0 || y >= height) return WALL;
        return cells[y * width + x];
    }

    CellType get(const Vec2i& pos) const { return get(pos.x, pos.y); }

    void set(int x, int y, CellType type) {
        if (x < 0 || x >= width || y < 0 || y >= height) return;
        cells[y * width + x] = type;
    }

    void set(const Vec2i& pos, CellType type) { set(pos.x, pos.y, type); }

    bool isWalkable(const Vec2i& pos) const {
        CellType type = get(pos);
        return type == EMPTY || type == START || type == END ||
            type == PATH || type == VISITED || type == FRONTIER;
    }

    Vec2i getStart() const { return start; }
    Vec2i getEnd() const { return end; }

    void setStart(const Vec2i& pos) {
        if (get(pos) != WALL) {
            set(start, EMPTY);
            start = pos;
            set(start, START);
        }
    }

    void setEnd(const Vec2i& pos) {
        if (get(pos) != WALL) {
            set(end, EMPTY);
            end = pos;
            set(end, END);
        }
    }

    void clear() {
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                CellType type = get(x, y);
                if (type == PATH || type == VISITED || type == FRONTIER) {
                    set(x, y, EMPTY);
                }
            }
        }
    }

    void clearAll() {
        for (int y = 1; y < height - 1; y++) {
            for (int x = 1; x < width - 1; x++) {
                set(x, y, EMPTY);
            }
        }
        set(start, START);
        set(end, END);
    }

    std::vector<Vec2i> getNeighbors(const Vec2i& pos, bool allowDiagonal = false) const {
        std::vector<Vec2i> neighbors;

        // 4-directional
        Vec2i dirs[] = { Vec2i(0, -1), Vec2i(1, 0), Vec2i(0, 1), Vec2i(-1, 0) };
        for (int i = 0; i < 4; i++) {
            Vec2i next = pos + dirs[i];
            if (isWalkable(next)) {
                neighbors.push_back(next);
            }
        }

        // Diagonal
        if (allowDiagonal) {
            Vec2i diags[] = { Vec2i(-1, -1), Vec2i(1, -1), Vec2i(1, 1), Vec2i(-1, 1) };
            for (int i = 0; i < 4; i++) {
                Vec2i next = pos + diags[i];
                if (isWalkable(next)) {
                    neighbors.push_back(next);
                }
            }
        }

        return neighbors;
    }
};

// =============================================================================
// ALGORITHM RESULT
// =============================================================================

struct AlgorithmResult {
    std::vector<Vec2i> path;
    std::unordered_set<Vec2i> visited;
    int nodesExplored;
    float pathCost;
    double timeMs;
    bool found;

    AlgorithmResult() : nodesExplored(0), pathCost(0), timeMs(0), found(false) {}
};

// =============================================================================
// PATHFINDING ALGORITHMS
// =============================================================================

class PathfindingAlgorithm {
protected:
    Grid* grid;

    std::vector<Vec2i> reconstructPath(const std::unordered_map<Vec2i, Vec2i>& cameFrom,
        Vec2i start, Vec2i end) {
        std::vector<Vec2i> path;
        Vec2i current = end;

        while (current != start) {
            path.push_back(current);
            std::unordered_map<Vec2i, Vec2i>::const_iterator it = cameFrom.find(current);
            if (it == cameFrom.end()) break;
            current = it->second;
        }
        path.push_back(start);
        std::reverse(path.begin(), path.end());
        return path;
    }

public:
    PathfindingAlgorithm(Grid* g) : grid(g) {}
    virtual ~PathfindingAlgorithm() {}
    virtual AlgorithmResult search() = 0;
    virtual const char* getName() const = 0;
};

// =============================================================================
// A* ALGORITHM
// =============================================================================

class AStarAlgorithm : public PathfindingAlgorithm {
    const Heuristic* heuristic;
    bool allowDiagonal;

    struct Node {
        Vec2i pos;
        float g, h, f;

        Node(Vec2i p, float g_, float h_) : pos(p), g(g_), h(h_), f(g_ + h_) {}

        bool operator>(const Node& other) const { return f > other.f; }
    };

public:
    AStarAlgorithm(Grid* g, const Heuristic* h, bool diagonal = false)
        : PathfindingAlgorithm(g), heuristic(h), allowDiagonal(diagonal) {
    }

    AlgorithmResult search() override {
        AlgorithmResult result;
        auto startTime = std::chrono::high_resolution_clock::now();

        Vec2i start = grid->getStart();
        Vec2i end = grid->getEnd();

        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;
        std::unordered_set<Vec2i> closedSet;
        std::unordered_map<Vec2i, Vec2i> cameFrom;
        std::unordered_map<Vec2i, float> gScore;

        gScore[start] = 0;
        openSet.push(Node(start, 0, heuristic->calculate(start, end)));

        while (!openSet.empty()) {
            Node current = openSet.top();
            openSet.pop();

            if (closedSet.find(current.pos) != closedSet.end()) continue;

            closedSet.insert(current.pos);
            result.visited.insert(current.pos);
            result.nodesExplored++;

            if (current.pos == end) {
                result.path = reconstructPath(cameFrom, start, end);
                result.pathCost = current.g;
                result.found = true;
                break;
            }

            std::vector<Vec2i> neighbors = grid->getNeighbors(current.pos, allowDiagonal);
            for (size_t i = 0; i < neighbors.size(); i++) {
                Vec2i neighbor = neighbors[i];

                if (closedSet.find(neighbor) != closedSet.end()) continue;

                float tentativeG = current.g + 1.0f;

                std::unordered_map<Vec2i, float>::iterator it = gScore.find(neighbor);
                if (it == gScore.end() || tentativeG < it->second) {
                    cameFrom[neighbor] = current.pos;
                    gScore[neighbor] = tentativeG;
                    float h = heuristic->calculate(neighbor, end);
                    openSet.push(Node(neighbor, tentativeG, h));
                }
            }
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        result.timeMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();

        return result;
    }

    const char* getName() const override { return "A*"; }
};

// =============================================================================
// DIJKSTRA ALGORITHM
// =============================================================================

class DijkstraAlgorithm : public PathfindingAlgorithm {
    struct Node {
        Vec2i pos;
        float cost;

        Node(Vec2i p, float c) : pos(p), cost(c) {}
        bool operator>(const Node& other) const { return cost > other.cost; }
    };

public:
    DijkstraAlgorithm(Grid* g) : PathfindingAlgorithm(g) {}

    AlgorithmResult search() override {
        AlgorithmResult result;
        auto startTime = std::chrono::high_resolution_clock::now();

        Vec2i start = grid->getStart();
        Vec2i end = grid->getEnd();

        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
        std::unordered_set<Vec2i> visited;
        std::unordered_map<Vec2i, Vec2i> cameFrom;
        std::unordered_map<Vec2i, float> dist;

        dist[start] = 0;
        pq.push(Node(start, 0));

        while (!pq.empty()) {
            Node current = pq.top();
            pq.pop();

            if (visited.find(current.pos) != visited.end()) continue;

            visited.insert(current.pos);
            result.visited.insert(current.pos);
            result.nodesExplored++;

            if (current.pos == end) {
                result.path = reconstructPath(cameFrom, start, end);
                result.pathCost = current.cost;
                result.found = true;
                break;
            }

            std::vector<Vec2i> neighbors = grid->getNeighbors(current.pos);
            for (size_t i = 0; i < neighbors.size(); i++) {
                Vec2i neighbor = neighbors[i];

                if (visited.find(neighbor) != visited.end()) continue;

                float newDist = current.cost + 1.0f;

                std::unordered_map<Vec2i, float>::iterator it = dist.find(neighbor);
                if (it == dist.end() || newDist < it->second) {
                    dist[neighbor] = newDist;
                    cameFrom[neighbor] = current.pos;
                    pq.push(Node(neighbor, newDist));
                }
            }
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        result.timeMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();

        return result;
    }

    const char* getName() const override { return "Dijkstra"; }
};

// =============================================================================
// BFS ALGORITHM
// =============================================================================

class BFSAlgorithm : public PathfindingAlgorithm {
public:
    BFSAlgorithm(Grid* g) : PathfindingAlgorithm(g) {}

    AlgorithmResult search() override {
        AlgorithmResult result;
        auto startTime = std::chrono::high_resolution_clock::now();

        Vec2i start = grid->getStart();
        Vec2i end = grid->getEnd();

        std::queue<Vec2i> q;
        std::unordered_set<Vec2i> visited;
        std::unordered_map<Vec2i, Vec2i> cameFrom;

        q.push(start);
        visited.insert(start);

        while (!q.empty()) {
            Vec2i current = q.front();
            q.pop();

            result.visited.insert(current);
            result.nodesExplored++;

            if (current == end) {
                result.path = reconstructPath(cameFrom, start, end);
                result.pathCost = (float)(result.path.size() - 1);
                result.found = true;
                break;
            }

            std::vector<Vec2i> neighbors = grid->getNeighbors(current);
            for (size_t i = 0; i < neighbors.size(); i++) {
                Vec2i neighbor = neighbors[i];

                if (visited.find(neighbor) == visited.end()) {
                    visited.insert(neighbor);
                    cameFrom[neighbor] = current;
                    q.push(neighbor);
                }
            }
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        result.timeMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();

        return result;
    }

    const char* getName() const override { return "BFS"; }
};

// =============================================================================
// DFS ALGORITHM
// =============================================================================

class DFSAlgorithm : public PathfindingAlgorithm {
public:
    DFSAlgorithm(Grid* g) : PathfindingAlgorithm(g) {}

    AlgorithmResult search() override {
        AlgorithmResult result;
        auto startTime = std::chrono::high_resolution_clock::now();

        Vec2i start = grid->getStart();
        Vec2i end = grid->getEnd();

        std::stack<Vec2i> stack;
        std::unordered_set<Vec2i> visited;
        std::unordered_map<Vec2i, Vec2i> cameFrom;

        stack.push(start);

        while (!stack.empty()) {
            Vec2i current = stack.top();
            stack.pop();

            if (visited.find(current) != visited.end()) continue;

            visited.insert(current);
            result.visited.insert(current);
            result.nodesExplored++;

            if (current == end) {
                result.path = reconstructPath(cameFrom, start, end);
                result.pathCost = (float)(result.path.size() - 1);
                result.found = true;
                break;
            }

            std::vector<Vec2i> neighbors = grid->getNeighbors(current);
            for (size_t i = 0; i < neighbors.size(); i++) {
                Vec2i neighbor = neighbors[i];

                if (visited.find(neighbor) == visited.end()) {
                    cameFrom[neighbor] = current;
                    stack.push(neighbor);
                }
            }
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        result.timeMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();

        return result;
    }

    const char* getName() const override { return "DFS"; }
};

// =============================================================================
// VISUALIZATION
// =============================================================================

class Visualizer {
    Grid* grid;
    int cellSize;
    int offsetX, offsetY;
    AlgorithmResult currentResult;

public:
    Visualizer(Grid* g, int cs = 20) : grid(g), cellSize(cs), offsetX(10), offsetY(80) {}

    void setResult(const AlgorithmResult& result) {
        currentResult = result;
    }

    void draw() {
        // Draw grid
        for (int y = 0; y < grid->getHeight(); y++) {
            for (int x = 0; x < grid->getWidth(); x++) {
                int px = offsetX + x * cellSize;
                int py = offsetY + y * cellSize;

                Color color = WHITE;
                Vec2i pos(x, y);
                CellType type = grid->get(pos);

                if (type == WALL) {
                    color = Color{ 40, 40, 40, 255 };
                }
                else if (type == START) {
                    color = GREEN;
                }
                else if (type == END) {
                    color = RED;
                }
                else if (currentResult.visited.find(pos) != currentResult.visited.end()) {
                    color = Color{ 100, 150, 255, 255 };
                }

                // Check if part of path
                bool inPath = false;
                for (size_t i = 0; i < currentResult.path.size(); i++) {
                    if (currentResult.path[i] == pos) {
                        inPath = true;
                        break;
                    }
                }

                if (inPath && type != START && type != END) {
                    color = YELLOW;
                }

                DrawRectangle(px, py, cellSize, cellSize, color);
                DrawRectangleLines(px, py, cellSize, cellSize, Color{ 200, 200, 200, 255 });
            }
        }
    }

    Vec2i screenToGrid(int mouseX, int mouseY) {
        int x = (mouseX - offsetX) / cellSize;
        int y = (mouseY - offsetY) / cellSize;
        return Vec2i(x, y);
    }
};

// =============================================================================
// APPLICATION
// =============================================================================

class Application {
    Grid grid;
    Visualizer visualizer;

    std::vector<PathfindingAlgorithm*> algorithms;
    std::vector<Heuristic*> heuristics;

    int currentAlgoIndex;
    int currentHeuristicIndex;
    AlgorithmResult lastResult;

    enum EditMode { MODE_WALL, MODE_START, MODE_END };
    EditMode editMode;

public:
    Application() : grid(60, 34), visualizer(&grid, 20),
        currentAlgoIndex(0), currentHeuristicIndex(0), editMode(MODE_WALL) {

        // Create heuristics
        heuristics.push_back(new ManhattanHeuristic());
        heuristics.push_back(new EuclideanHeuristic());
        heuristics.push_back(new DiagonalHeuristic());

        // Create algorithms
        algorithms.push_back(new AStarAlgorithm(&grid, heuristics[0]));
        algorithms.push_back(new DijkstraAlgorithm(&grid));
        algorithms.push_back(new BFSAlgorithm(&grid));
        algorithms.push_back(new DFSAlgorithm(&grid));

        // Add some walls for demo
        generateMaze();
    }

    ~Application() {
        for (size_t i = 0; i < algorithms.size(); i++) delete algorithms[i];
        for (size_t i = 0; i < heuristics.size(); i++) delete heuristics[i];
    }

    void run() {
        InitWindow(1280, 800, "Pathfinding & Optimization Toolkit");
        SetTargetFPS(60);

        while (!WindowShouldClose()) {
            update();
            render();
        }

        CloseWindow();
    }

private:
    void generateMaze() {
        // Simple random maze
        for (int y = 5; y < grid.getHeight() - 5; y++) {
            for (int x = 5; x < grid.getWidth() - 5; x++) {
                if (GetRandomValue(0, 100) < 20) {
                    grid.set(x, y, WALL);
                }
            }
        }
    }

    void update() {
        // Mouse input for editing
        if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
            Vec2i gridPos = visualizer.screenToGrid(GetMouseX(), GetMouseY());

            if (editMode == MODE_WALL) {
                if (grid.get(gridPos) != START && grid.get(gridPos) != END) {
                    grid.set(gridPos, WALL);
                }
            }
            else if (editMode == MODE_START) {
                grid.setStart(gridPos);
            }
            else if (editMode == MODE_END) {
                grid.setEnd(gridPos);
            }
        }

        if (IsMouseButtonDown(MOUSE_RIGHT_BUTTON)) {
            Vec2i gridPos = visualizer.screenToGrid(GetMouseX(), GetMouseY());
            if (grid.get(gridPos) == WALL) {
                grid.set(gridPos, EMPTY);
            }
        }

        // Keyboard controls
        if (IsKeyPressed(KEY_SPACE)) {
            runCurrentAlgorithm();
        }

        if (IsKeyPressed(KEY_C)) {
            grid.clear();
            lastResult = AlgorithmResult();
            visualizer.setResult(lastResult);
        }

        if (IsKeyPressed(KEY_R)) {
            grid.clearAll();
            generateMaze();
            lastResult = AlgorithmResult();
            visualizer.setResult(lastResult);
        }

        if (IsKeyPressed(KEY_ONE)) currentAlgoIndex = 0;
        if (IsKeyPressed(KEY_TWO)) currentAlgoIndex = 1;
        if (IsKeyPressed(KEY_THREE)) currentAlgoIndex = 2;
        if (IsKeyPressed(KEY_FOUR)) currentAlgoIndex = 3;

        if (IsKeyPressed(KEY_H)) {
            currentHeuristicIndex = (currentHeuristicIndex + 1) % (int)heuristics.size();
            delete algorithms[0];
            algorithms[0] = new AStarAlgorithm(&grid, heuristics[currentHeuristicIndex]);
        }

        if (IsKeyPressed(KEY_W)) editMode = MODE_WALL;
        if (IsKeyPressed(KEY_S)) editMode = MODE_START;
        if (IsKeyPressed(KEY_E)) editMode = MODE_END;

        if (IsKeyPressed(KEY_A)) {
            runAllAlgorithms();
        }
    }

    void runCurrentAlgorithm() {
        grid.clear();
        lastResult = algorithms[currentAlgoIndex]->search();
        visualizer.setResult(lastResult);
    }

    void runAllAlgorithms() {
        std::stringstream ss;
        ss << "\n=== PERFORMANCE COMPARISON ===\n";
        ss << std::fixed << std::setprecision(4);

        for (size_t i = 0; i < algorithms.size(); i++) {
            grid.clear();
            AlgorithmResult result = algorithms[i]->search();

            ss << algorithms[i]->getName() << ":\n";
            ss << "  Time: " << result.timeMs << " ms\n";
            ss << "  Nodes: " << result.nodesExplored << "\n";
            ss << "  Path: " << result.path.size() << " steps\n";
            ss << "  Cost: " << result.pathCost << "\n";
            ss << "  Found: " << (result.found ? "YES" : "NO") << "\n\n";
        }

        TraceLog(LOG_INFO, ss.str().c_str());
    }

    void render() {
        BeginDrawing();
        ClearBackground(Color{ 30, 30, 30, 255 });

        // Draw UI
        DrawText("PATHFINDING & OPTIMIZATION TOOLKIT", 10, 10, 24, RAYWHITE);
        DrawText("Press SPACE to run algorithm | C = Clear | R = Reset | A = Compare All", 10, 40, 16, GRAY);
        DrawText("Press 1/2/3/4 to choose algorithm", 10, 55, 16, GRAY);

        // Controls info
        const char* algoName = algorithms[currentAlgoIndex]->getName();
        DrawText(TextFormat("Algorithm [1-4]: %s", algoName), 10, 720, 18, YELLOW);

        if (currentAlgoIndex == 0) {
            DrawText(TextFormat("Heuristic [H]: %s", heuristics[currentHeuristicIndex]->getName()),
                10, 745, 18, YELLOW);
        }

        DrawText("Edit Mode [W/S/E]: ", 600, 720, 18, WHITE);
        Color modeColor = editMode == MODE_WALL ? RED : (editMode == MODE_START ? GREEN : BLUE);
        const char* modeText = editMode == MODE_WALL ? "WALL" : (editMode == MODE_START ? "START" : "END");
        DrawText(modeText, 790, 720, 18, modeColor);

        // Stats
        if (lastResult.found) {
            DrawText(TextFormat("Time: %.4f ms", lastResult.timeMs), 600, 745, 18, WHITE);
            DrawText(TextFormat("Nodes: %d", lastResult.nodesExplored), 800, 745, 18, WHITE);
            DrawText(TextFormat("Path: %d steps", (int)lastResult.path.size()), 950, 745, 18, WHITE);
        }

        // Legend
        DrawRectangle(900, 10, 20, 20, GREEN);
        DrawText("Start", 925, 10, 16, WHITE);
        DrawRectangle(900, 35, 20, 20, RED);
        DrawText("End", 925, 35, 16, WHITE);
        DrawRectangle(1050, 10, 20, 20, Color{ 100, 150, 255, 255 });
        DrawText("Visited", 1075, 10, 16, WHITE);
        DrawRectangle(1050, 35, 20, 20, YELLOW);
        DrawText("Path", 1075, 35, 16, WHITE);

        // Draw visualization
        visualizer.draw();

        EndDrawing();
    }
};

// =============================================================================
// MAIN
// =============================================================================

int main() {
    Application app;
    app.run();
    return 0;
}