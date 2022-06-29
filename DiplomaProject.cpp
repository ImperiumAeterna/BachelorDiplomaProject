#include <iostream>
#include <iomanip>
#include <unordered_map>
#include <unordered_set>
#include <array>
#include <vector>
#include <utility>
#include <queue>
#include <tuple>
#include <algorithm>
#include <cstdlib>
#include <chrono>
#include <time.h>
#define HEUR 25
#define SIZE 10 //not less than 10
typedef std::chrono::steady_clock::time_point tp; 


class Time {
public:
    static void show(tp t1, tp t2);
    tp addPoint();
};

struct GridLocation;

bool same_grid(GridLocation l1, GridLocation l2);

int in_visited(std::vector<GridLocation> visitedGrid, GridLocation loc);

struct SquareGrid;

std::array<GridLocation, 4> SquareGrid::DIRS = {
    GridLocation{1, 0}, GridLocation{-1, 0},
    GridLocation{0, -1}, GridLocation{0, 1}
};

bool operator == (GridLocation a, GridLocation b);
bool operator != (GridLocation a, GridLocation b);
bool operator < (GridLocation a, GridLocation b);

std::basic_iostream<char>::basic_ostream& operator<<
    (std::basic_iostream<char>::basic_ostream& out, const GridLocation& loc);

template<class Graph>
void draw_grid(const Graph& graph,
    std::unordered_map<GridLocation, double>* distances = nullptr,
    std::unordered_map<GridLocation, GridLocation>* point_to = nullptr,
    std::vector<GridLocation>* path = nullptr,
    GridLocation* start = nullptr,
    GridLocation* goal = nullptr);

void add_rect(SquareGrid& grid, int x1, int y1, int x2, int y2);

SquareGrid make_diagram1();

struct GridWithWeights : SquareGrid;

GridWithWeights make_diagram4(std::unordered_set<GridLocation> tmp = {});
GridWithWeights edit_diagram4(GridWithWeights& grid, std::vector <GridLocation> path);

template<typename T, typename priority_t>
struct PriorityQueue;

template<typename Location, typename Graph>
void dijkstra_search
(Graph graph,
    Location start,
    Location goal,
    std::unordered_map<Location, Location>& came_from,
    std::unordered_map<Location, double>& cost_so_far);

template<typename Location> std::vector<Location> reconstruct_path(
    Location start, Location goal,
    std::unordered_map<Location, Location> came_from);

void add_draw_agent(int s1, int s2, int d1, int d2, GridWithWeights& grid);

namespace std {
    template <> struct hash<GridLocation> {
        std::size_t operator()(const GridLocation& id) const noexcept {
            return std::hash<int>()(id.x ^ (id.y << 16));
        }
    };
}

int main() {
    Time time;
    GridWithWeights grid = make_diagram4();
    std::vector<GridLocation> visitedGrid = {};
    std::vector<int> visitedInt = {};
    GridLocation start{1, 4}, goal{8, 3};
    std::unordered_map<GridLocation, GridLocation> came_from;
    std::unordered_map<GridLocation, double> cost_so_far;
    auto t1 = time.addPoint();
    dijkstra_search(grid, start, goal, came_from, cost_so_far);
    auto t2 = time.addPoint();
    draw_grid(grid, nullptr, &came_from, nullptr, &start, &goal);
    std::cout << '\n';
    std::vector<GridLocation> path = reconstruct_path(start, goal, came_from);
    draw_grid(grid, nullptr, nullptr, &path, &start, &goal);
    std::cout << '\n';
    draw_grid(grid, &cost_so_far, nullptr, nullptr, &start, &goal);
    auto t3 = time.addPoint();
    grid = edit_diagram4(grid, path);//new
    auto t4 = time.addPoint();
    add_draw_agent(1, 0, 9, 8, grid);
    auto t5 = time.addPoint();
    for (auto i = 0; i < HEUR; i++)
        grid.turn();
    auto t6 = time.addPoint();
    auto t7 = time.addPoint();
    grid.turn(25);
    auto t8 = time.addPoint();
    add_draw_agent(2, 5, 7, 8, grid);
    
    time.show(t1, t2);
    time.show(t3, t4);
    time.show(t5, t6);
    time.show(t7, t8);
    return 0;
}


class Time {
public:
    static void show(tp t1, tp t2) {
        std::cout << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << '\t';
        printf("microseconds \n");
    }
    tp addPoint() {
        tp p = std::chrono::steady_clock::now();
        return p;
    }
};
struct GridLocation {
    int x, y;
    int xGridLocation() { return this->x; }
    int yGridLocation() { return this->y; }
};

bool same_grid(GridLocation l1, GridLocation l2) {
    return (l1.xGridLocation() == l2.xGridLocation()
        && l1.xGridLocation() == l2.yGridLocation());
}
int in_visited(std::vector<GridLocation> visitedGrid, GridLocation loc) {
    for (auto i = 0; i < visitedGrid.size(); i++)
    {
        if (same_grid(visitedGrid[i], loc))
            return i;
    }
    return -1;
}
namespace std {
    template <> struct hash<GridLocation> {
        std::size_t operator()(const GridLocation& id) const noexcept {
            return std::hash<int>()(id.x ^ (id.y << 16));
        }
    };
}


struct SquareGrid {
    static std::array<GridLocation, 4> DIRS;

    int width, height;
    std::unordered_set<GridLocation> walls;

    SquareGrid(int width_, int height_)
        : width(width_), height(height_) {}

    bool in_bounds(GridLocation id) const {
        return 0 <= id.x && id.x < width
            && 0 <= id.y && id.y < height;
    }

    bool passable(GridLocation id) const {
        return walls.find(id) == walls.end();
    }

    std::vector<GridLocation> neighbors(GridLocation id) const {
        std::vector<GridLocation> results;

        for (GridLocation dir : DIRS) {
            GridLocation next{ id.x + dir.x, id.y + dir.y };
            if (in_bounds(next) && passable(next)) {
                results.push_back(next);
            }
        }

        if ((id.x + id.y) % 2 == 0) {
            std::reverse(results.begin(), results.end());
        }

        return results;
    }
};

std::array<GridLocation, 4> SquareGrid::DIRS = {
    GridLocation{1, 0}, GridLocation{-1, 0},
    GridLocation{0, -1}, GridLocation{0, 1}
};

bool operator == (GridLocation a, GridLocation b) {
    return a.x == b.x && a.y == b.y;
}

bool operator != (GridLocation a, GridLocation b) {
    return !(a == b);
}

bool operator < (GridLocation a, GridLocation b) {
    return std::tie(a.x, a.y) < std::tie(b.x, b.y);
}

std::basic_iostream<char>::basic_ostream& operator<<(std::basic_iostream<char>::basic_ostream& out, const GridLocation& loc) {
    out << '(' << loc.x << ',' << loc.y << ')';
    return out;
}
template<class Graph>
void draw_grid(const Graph& graph,
    std::unordered_map<GridLocation, double>* distances = nullptr,
    std::unordered_map<GridLocation, GridLocation>* point_to = nullptr,
    std::vector<GridLocation>* path = nullptr,
    GridLocation* start = nullptr,
    GridLocation* goal = nullptr) {
    const int field_width = 3;
    std::cout << std::string(field_width * graph.width, '_') << '\n';
    for (int y = 0; y != graph.height; ++y) {
        for (int x = 0; x != graph.width; ++x) {
            GridLocation id{ x, y };
            if (graph.walls.find(id) != graph.walls.end()) {
                std::cout << std::string(field_width, '#');
            }
            else if (start && id == *start) {
                std::cout << " A ";
            }
            else if (goal && id == *goal) {
                std::cout << " Z ";
            }
            else if (path != nullptr && find(path->begin(), path->end(), id) != path->end()) {
                std::cout << " @ ";
            }
            else if (point_to != nullptr && point_to->count(id)) {
                GridLocation next = (*point_to)[id];
                if (next.x == x + 1) { std::cout << " > "; }
                else if (next.x == x - 1) { std::cout << " < "; }
                else if (next.y == y + 1) { std::cout << " v "; }
                else if (next.y == y - 1) { std::cout << " ^ "; }
                else { std::cout << " * "; }
            }
            else if (distances != nullptr && distances->count(id)) {
                std::cout << ' ' << std::left << std::setw(field_width - 1) << (*distances)[id];
            }
            else {
                std::cout << " . ";
            }
        }
        std::cout << '\n';
    }
    std::cout << std::string(field_width * graph.width, '~') << '\n';
}

void add_rect(SquareGrid& grid, int x1, int y1, int x2, int y2) {
    for (int x = x1; x < x2; ++x) {
        for (int y = y1; y < y2; ++y) {
            grid.walls.insert(GridLocation{ x, y });
        }
    }
}

SquareGrid make_diagram1() {
    SquareGrid grid(30, 15);
    add_rect(grid, 3, 3, 5, 12);
    add_rect(grid, 13, 4, 15, 15);
    add_rect(grid, 21, 0, 23, 7);
    add_rect(grid, 23, 5, 26, 7);
    return grid;
}

struct GridWithWeights : SquareGrid {
    int weights[SIZE][SIZE] = {};
    int cw[SIZE][SIZE] = {}; //const weights
    GridWithWeights(int w, int h) : SquareGrid(w, h) {}
    double cost(GridLocation from_node, GridLocation to_node) const {
        return weights[to_node.yGridLocation()][to_node.xGridLocation()]
            + cw[to_node.yGridLocation()][to_node.xGridLocation()];
    };
    void turn(int t = 1) {
        for (auto i = 0; i < SIZE; i++)
            for (auto j = 0; j < SIZE; j++)
                if (weights[i][j] > t)
                    weights[i][j] -= t;
    }
};

GridWithWeights make_diagram4(std::unordered_set<GridLocation> tmp = {}) {
    GridWithWeights grid(SIZE, SIZE);
    add_rect(grid, 1, 7, 4, 9);
    for (auto i = 0; i < SIZE; i++)
        for (auto j = 0; j < SIZE; j++)
            grid.weights[i][j] = 1;
    for (auto i = 3; i < 7; i++) {
        for (auto j = 4; j < 8; j++) {
            grid.cw[i][j] = 4; //forests with weights +4
        }
    }
    return grid;
}
GridWithWeights edit_diagram4(GridWithWeights& grid, std::vector <GridLocation> path)
{
    typedef GridLocation L;
    bool arr[SIZE][SIZE] = { 0 };
    for (auto i : path) {
        arr[i.yGridLocation()][i.xGridLocation()] = true;
    }
    int c = 0;
    std::unordered_set <GridLocation> tmp = {};
    for (auto i = 0; i < SIZE; i++)
        for (auto j = 0; j < SIZE; j++) {
            if (arr[i][j])
                grid.weights[i][j] += HEUR;
        }
    return grid;
}

template<typename T, typename priority_t>
struct PriorityQueue {
    typedef std::pair<priority_t, T> PQElement;
    std::priority_queue<PQElement, std::vector<PQElement>,
        std::greater<PQElement>> elements;

    inline bool empty() const {
        return elements.empty();
    }

    inline void put(T item, priority_t priority) {
        elements.emplace(priority, item);
    }

    T get() {
        T best_item = elements.top().second;
        elements.pop();
        return best_item;
    }
};

template<typename Location, typename Graph>
void dijkstra_search
(Graph graph,
    Location start,
    Location goal,
    std::unordered_map<Location, Location>& came_from,
    std::unordered_map<Location, double>& cost_so_far)
{
    PriorityQueue<Location, double> frontier;
    frontier.put(start, 0);

    came_from[start] = start;
    cost_so_far[start] = 0;

    while (!frontier.empty()) {
        Location current = frontier.get();

        if (current == goal) {
            break;
        }

        for (Location next : graph.neighbors(current)) {
            double new_cost = cost_so_far[current] + graph.cost(current, next);
            if (cost_so_far.find(next) == cost_so_far.end()
                || new_cost < cost_so_far[next]) {
                cost_so_far[next] = new_cost;
                came_from[next] = current;
                frontier.put(next, new_cost);
            }
        }
    }
}

template<typename Location>
std::vector<Location> reconstruct_path(
    Location start, Location goal,
    std::unordered_map<Location, Location> came_from
) {
    std::vector<Location> path;
    Location current = goal;
    while (current != start) {  // note: this will fail if no path found
        path.push_back(current);
        current = came_from[current];
    }
    path.push_back(start); // optional
    std::reverse(path.begin(), path.end());
    return path;
}

void add_draw_agent(int s1, int s2, int d1, int d2, GridWithWeights& grid)
{
    GridLocation start{ s1,s2 }, goal{ d1,d2 };
    std::unordered_map<GridLocation, GridLocation> came_from;
    std::unordered_map<GridLocation, double> cost_so_far;
    dijkstra_search(grid, start, goal, came_from, cost_so_far);
    std::vector<GridLocation> path = reconstruct_path(start, goal, came_from);
    draw_grid(grid, nullptr, nullptr, &path, &start, &goal);
    draw_grid(grid, &cost_so_far, nullptr, nullptr, &start, &goal);
    edit_diagram4(grid, path);

};



