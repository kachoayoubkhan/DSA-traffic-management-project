#include <algorithm>
#include <cctype>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

using namespace std;

constexpr int INF = numeric_limits<int>::max() / 4;

// These helpers keep dataset parsing tolerant to spaces and user input variations.
string trim(const string& value) {
    const size_t start = value.find_first_not_of(" \t\r\n");
    if (start == string::npos) {
        return "";
    }

    const size_t end = value.find_last_not_of(" \t\r\n");
    return value.substr(start, end - start + 1);
}

string normalizeKey(string value) {
    value = trim(value);
    transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(tolower(ch));
    });
    return value;
}

vector<string> splitCsvRow(const string& line) {
    vector<string> cells;
    string cell;
    stringstream stream(line);

    while (getline(stream, cell, ',')) {
        cells.push_back(trim(cell));
    }

    return cells;
}

struct PathResult {
    vector<int> distances;
    vector<int> parents;
    int source = -1;

    bool isReachable(int destination) const {
        return destination >= 0 &&
               destination < static_cast<int>(distances.size()) &&
               distances[destination] < INF;
    }

    vector<int> buildPath(int destination) const {
        if (!isReachable(destination)) {
            return {};
        }

        vector<int> path;
        for (int current = destination; current != -1; current = parents[current]) {
            path.push_back(current);
        }

        reverse(path.begin(), path.end());
        return path;
    }
};

class Graph {
private:
    // The matrix is convenient for direct road queries; the list is faster for graph traversals.
    vector<vector<int>> adjacencyMatrix;
    vector<vector<pair<int, int>>> adjacencyList;
    vector<string> vertexNames;
    unordered_map<string, int> indexByName;

    void rebuildIndex() {
        indexByName.clear();

        for (int index = 0; index < static_cast<int>(vertexNames.size()); ++index) {
            vertexNames[index] = trim(vertexNames[index]);
            if (vertexNames[index].empty()) {
                throw runtime_error("Encountered an empty intersection name in the dataset.");
            }

            const string key = normalizeKey(vertexNames[index]);
            if (indexByName.count(key) != 0) {
                throw runtime_error("Duplicate intersection detected: " + vertexNames[index]);
            }

            indexByName[key] = index;
        }
    }

    void rebuildAdjacencyList() {
        adjacencyList.assign(vertexCount(), {});

        for (int from = 0; from < vertexCount(); ++from) {
            for (int to = 0; to < vertexCount(); ++to) {
                if (from != to && adjacencyMatrix[from][to] < INF) {
                    adjacencyList[from].push_back({to, adjacencyMatrix[from][to]});
                }
            }
        }
    }

public:
    Graph() = default;

    Graph(vector<string> names, vector<vector<int>> matrix)
        : adjacencyMatrix(std::move(matrix)), vertexNames(std::move(names)) {
        const int n = static_cast<int>(vertexNames.size());

        if (n == 0) {
            throw runtime_error("The graph cannot be empty.");
        }

        if (static_cast<int>(adjacencyMatrix.size()) != n) {
            throw runtime_error("The adjacency matrix size does not match the number of intersections.");
        }

        for (const auto& row : adjacencyMatrix) {
            if (static_cast<int>(row.size()) != n) {
                throw runtime_error("The adjacency matrix must be square.");
            }
        }

        rebuildIndex();
        rebuildAdjacencyList();
    }

    static Graph fromCsv(const string& fileName) {
        // The CSV layout is: header row of intersections, then one weighted row per source intersection.
        ifstream file(fileName);
        if (!file.is_open()) {
            throw runtime_error("Could not open file: " + fileName);
        }

        string headerLine;
        if (!getline(file, headerLine)) {
            throw runtime_error("CSV file is empty: " + fileName);
        }

        const vector<string> headerCells = splitCsvRow(headerLine);
        if (headerCells.size() < 2) {
            throw runtime_error("CSV header is incomplete in file: " + fileName);
        }

        vector<string> names(headerCells.begin() + 1, headerCells.end());
        const int n = static_cast<int>(names.size());
        vector<vector<int>> matrix(n, vector<int>(n, INF));

        for (int index = 0; index < n; ++index) {
            matrix[index][index] = 0;
        }

        string line;
        int rowIndex = 0;
        while (getline(file, line)) {
            if (trim(line).empty()) {
                continue;
            }

            const vector<string> cells = splitCsvRow(line);
            if (static_cast<int>(cells.size()) != n + 1) {
                throw runtime_error("Malformed row in file: " + fileName);
            }

            if (rowIndex >= n) {
                throw runtime_error("File contains more rows than expected: " + fileName);
            }

            if (normalizeKey(cells[0]) != normalizeKey(names[rowIndex])) {
                throw runtime_error("Row and column labels do not align in file: " + fileName);
            }

            for (int col = 0; col < n; ++col) {
                int value = 0;
                try {
                    value = stoi(cells[col + 1]);
                } catch (const exception&) {
                    throw runtime_error("Non-numeric weight encountered in file: " + fileName);
                }

                if (rowIndex == col) {
                    matrix[rowIndex][col] = 0;
                } else if (value == 999) {
                    matrix[rowIndex][col] = INF;
                } else {
                    if (value < 0) {
                        throw runtime_error("Negative edge weights are not supported by Dijkstra's algorithm.");
                    }
                    matrix[rowIndex][col] = value;
                }
            }

            ++rowIndex;
        }

        if (rowIndex != n) {
            throw runtime_error("The number of data rows does not match the header in file: " + fileName);
        }

        return Graph(std::move(names), std::move(matrix));
    }

    int vertexCount() const {
        return static_cast<int>(vertexNames.size());
    }

    bool contains(const string& name) const {
        return indexByName.count(normalizeKey(name)) != 0;
    }

    int indexOf(const string& name) const {
        const auto it = indexByName.find(normalizeKey(name));
        if (it == indexByName.end()) {
            throw invalid_argument("Intersection not found: " + trim(name));
        }
        return it->second;
    }

    const string& vertexName(int index) const {
        return vertexNames.at(index);
    }

    const vector<string>& names() const {
        return vertexNames;
    }

    bool hasEdge(int from, int to) const {
        return from >= 0 &&
               from < vertexCount() &&
               to >= 0 &&
               to < vertexCount() &&
               from != to &&
               adjacencyMatrix[from][to] < INF;
    }

    int weight(int from, int to) const {
        return adjacencyMatrix.at(from).at(to);
    }

    vector<int> blockedNeighbors(int vertex) const {
        vector<int> blocked;

        for (int other = 0; other < vertexCount(); ++other) {
            if (vertex != other && adjacencyMatrix[vertex][other] >= INF) {
                blocked.push_back(other);
            }
        }

        return blocked;
    }

    PathResult dijkstra(int source) const {
        // Dijkstra with a min-heap gives a stronger implementation than the O(V^2) array scan.
        vector<int> distances(vertexCount(), INF);
        vector<int> parents(vertexCount(), -1);
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> frontier;

        // The adjacency matrix is useful for direct road lookups; the adjacency list keeps Dijkstra efficient.
        distances[source] = 0;
        frontier.push({0, source});

        while (!frontier.empty()) {
            const pair<int, int> current = frontier.top();
            frontier.pop();

            const int currentDistance = current.first;
            const int node = current.second;

            if (currentDistance != distances[node]) {
                continue;
            }

            for (const auto& edge : adjacencyList[node]) {
                const int neighbor = edge.first;
                const int weightValue = edge.second;
                const long long candidateDistance =
                    static_cast<long long>(currentDistance) + weightValue;

                if (candidateDistance < distances[neighbor]) {
                    distances[neighbor] = static_cast<int>(candidateDistance);
                    parents[neighbor] = node;
                    frontier.push({distances[neighbor], neighbor});
                }
            }
        }

        return {distances, parents, source};
    }

    vector<vector<int>> floydWarshall() const {
        // Floyd-Warshall gives us network-wide analytics such as the graph center and diameter.
        vector<vector<int>> distances = adjacencyMatrix;

        for (int via = 0; via < vertexCount(); ++via) {
            for (int from = 0; from < vertexCount(); ++from) {
                if (distances[from][via] >= INF) {
                    continue;
                }

                for (int to = 0; to < vertexCount(); ++to) {
                    if (distances[via][to] >= INF) {
                        continue;
                    }

                    const int candidate = distances[from][via] + distances[via][to];
                    if (candidate < distances[from][to]) {
                        distances[from][to] = candidate;
                    }
                }
            }
        }

        return distances;
    }
};

int greenSignalMinutes(int intensity) {
    if (intensity < 100) {
        return 3;
    }
    if (intensity < 300) {
        return 4;
    }
    if (intensity < 600) {
        return 5;
    }
    return 6;
}

int minutesPerKilometer(int intensity) {
    if (intensity > 600) {
        return 9;
    }
    if (intensity >= 300) {
        return 7;
    }
    if (intensity >= 100) {
        return 4;
    }
    return 2;
}

int edgeTravelTime(int distance, int intensity) {
    return distance * minutesPerKilometer(intensity);
}

void ensureCompatibleGraphs(const Graph& distanceGraph, const Graph& intensityGraph) {
    if (distanceGraph.names() != intensityGraph.names()) {
        throw runtime_error("Distance and traffic datasets do not use the same intersections.");
    }
}

Graph buildTravelTimeGraph(const Graph& distanceGraph, const Graph& intensityGraph) {
    ensureCompatibleGraphs(distanceGraph, intensityGraph);

    const int n = distanceGraph.vertexCount();
    vector<vector<int>> matrix(n, vector<int>(n, INF));

    for (int index = 0; index < n; ++index) {
        matrix[index][index] = 0;
    }

    for (int from = 0; from < n; ++from) {
        for (int to = 0; to < n; ++to) {
            if (!distanceGraph.hasEdge(from, to) || !intensityGraph.hasEdge(from, to)) {
                continue;
            }

            // Each edge stores travel time, so the shortest-path result becomes the fastest route.
            matrix[from][to] = edgeTravelTime(
                distanceGraph.weight(from, to),
                intensityGraph.weight(from, to)
            );
        }
    }

    return Graph(distanceGraph.names(), std::move(matrix));
}

int readMenuChoice() {
    // Input validation keeps the menu from breaking on accidental text input.
    while (true) {
        cout << "\nEnter your choice: ";

        int choice = 0;
        if (cin >> choice) {
            cin.ignore(numeric_limits<streamsize>::max(), '\n');
            return choice;
        }

        cout << "Please enter a valid numeric menu option.\n";
        cin.clear();
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
    }
}

void printIntersections(const Graph& graph) {
    cout << "Available intersections:\n";
    for (const string& name : graph.names()) {
        cout << "  - " << name << '\n';
    }
}

string promptIntersection(const Graph& graph, const string& prompt) {
    while (true) {
        cout << prompt;
        string input;
        getline(cin, input);
        input = trim(input);

        if (graph.contains(input)) {
            return input;
        }

        cout << "Intersection not found. Please choose a name from the list above.\n";
    }
}

void printPath(const Graph& graph, const vector<int>& path) {
    for (int index = 0; index < static_cast<int>(path.size()); ++index) {
        cout << graph.vertexName(path[index]);
        if (index + 1 < static_cast<int>(path.size())) {
            cout << " -> ";
        }
    }
    cout << '\n';
}

int pathWeight(const Graph& graph, const vector<int>& path) {
    int total = 0;

    for (int index = 0; index + 1 < static_cast<int>(path.size()); ++index) {
        total += graph.weight(path[index], path[index + 1]);
    }

    return total;
}

void printSegmentBreakdown(
    const Graph& distanceGraph,
    const Graph& intensityGraph,
    const vector<int>& path
) {
    if (path.size() < 2) {
        cout << "Segment breakdown: source and destination are the same intersection.\n";
        return;
    }

    cout << "Segment breakdown:\n";
    // Showing each edge separately makes the traffic-aware route easy to justify in a demo or viva.
    for (int index = 0; index + 1 < static_cast<int>(path.size()); ++index) {
        const int from = path[index];
        const int to = path[index + 1];
        const int distance = distanceGraph.weight(from, to);
        const int intensity = intensityGraph.weight(from, to);

        cout << "  " << index + 1 << ". "
             << distanceGraph.vertexName(from) << " -> " << distanceGraph.vertexName(to)
             << " | distance: " << distance << " km"
             << " | traffic: " << intensity << " vehicles"
             << " | time: " << edgeTravelTime(distance, intensity) << " minutes\n";
    }
}

void printBlockedRoadsOnRoute(const Graph& graph, const vector<int>& path) {
    bool foundBlockedRoad = false;
    cout << "Blocked roads touching this route:\n";

    for (int node : path) {
        for (int blockedNeighbor : graph.blockedNeighbors(node)) {
            foundBlockedRoad = true;
            cout << "  - " << graph.vertexName(node)
                 << " -> " << graph.vertexName(blockedNeighbor)
                 << " is blocked in the current dataset.\n";
        }
    }

    if (!foundBlockedRoad) {
        cout << "  None. Every outgoing road on this corridor is open.\n";
    }
}

void printMenu() {
    cout << "\n--- Digital Traffic Management System ---\n";
    cout << "1. Find VIP Route (Priority-Queue Dijkstra)\n";
    cout << "2. Inspect Direct Road Traffic\n";
    cout << "3. Display Green Signal Plan\n";
    cout << "4. Calculate Fastest Route (Traffic-Aware Dijkstra)\n";
    cout << "5. Show Network Summary (Floyd-Warshall)\n";
    cout << "6. Exit\n";
}

void handleVipRoute(const Graph& distanceGraph) {
    printIntersections(distanceGraph);
    const string startName = promptIntersection(distanceGraph, "\nEnter the starting intersection: ");
    const string endName = promptIntersection(distanceGraph, "Enter the destination intersection: ");

    const int start = distanceGraph.indexOf(startName);
    const int end = distanceGraph.indexOf(endName);
    const PathResult result = distanceGraph.dijkstra(start);

    if (!result.isReachable(end)) {
        cout << "No VIP route exists between " << startName << " and " << endName << ".\n";
        return;
    }

    const vector<int> path = result.buildPath(end);
    cout << "\nVIP Route Summary\n";
    cout << "Algorithm used: Priority-Queue Dijkstra\n";
    cout << "Shortest distance: " << result.distances[end] << " km\n";
    cout << "Route: ";
    printPath(distanceGraph, path);
    printBlockedRoadsOnRoute(distanceGraph, path);
}

void handleDirectTraffic(const Graph& distanceGraph, const Graph& intensityGraph) {
    printIntersections(intensityGraph);
    const string startName = promptIntersection(intensityGraph, "\nEnter the source intersection: ");
    const string endName = promptIntersection(intensityGraph, "Enter the destination intersection: ");

    const int start = intensityGraph.indexOf(startName);
    const int end = intensityGraph.indexOf(endName);

    // This option intentionally reports only a direct road. If none exists, it suggests a shortest detour.
    if (!distanceGraph.hasEdge(start, end) || !intensityGraph.hasEdge(start, end)) {
        cout << "There is no direct road between " << startName << " and " << endName << ".\n";

        const PathResult detour = distanceGraph.dijkstra(start);
        if (detour.isReachable(end)) {
            cout << "Suggested multi-hop corridor by shortest distance: ";
            printPath(distanceGraph, detour.buildPath(end));
        }
        return;
    }

    const int traffic = intensityGraph.weight(start, end);
    cout << "\nDirect Road Analytics\n";
    cout << "Vehicles expected on " << startName << " -> " << endName << ": " << traffic << '\n';
    cout << "Distance of this road: " << distanceGraph.weight(start, end) << " km\n";
    cout << "Suggested green signal duration: " << greenSignalMinutes(traffic) << " minutes\n";
}

void handleGreenSignalPlan(const Graph& intensityGraph) {
    cout << "\nGreen Signal Plan\n";
    cout << "Each open road segment is mapped to a signal duration using traffic thresholds.\n";

    for (int from = 0; from < intensityGraph.vertexCount(); ++from) {
        cout << '\n' << intensityGraph.vertexName(from) << ":\n";

        for (int to = 0; to < intensityGraph.vertexCount(); ++to) {
            if (!intensityGraph.hasEdge(from, to)) {
                continue;
            }

            const int traffic = intensityGraph.weight(from, to);
            cout << "  -> " << intensityGraph.vertexName(to)
                 << " | traffic: " << traffic
                 << " | green signal: " << greenSignalMinutes(traffic) << " minutes\n";
        }
    }
}

void handleFastestRoute(
    const Graph& distanceGraph,
    const Graph& intensityGraph,
    const Graph& travelTimeGraph
) {
    printIntersections(distanceGraph);
    const string startName = promptIntersection(distanceGraph, "\nEnter the starting intersection: ");
    const string endName = promptIntersection(distanceGraph, "Enter the destination intersection: ");

    const int start = distanceGraph.indexOf(startName);
    const int end = distanceGraph.indexOf(endName);
    // Dijkstra runs on the derived travel-time graph rather than raw distance or raw traffic values alone.
    const PathResult result = travelTimeGraph.dijkstra(start);

    if (!result.isReachable(end)) {
        cout << "No valid traffic-aware route exists between " << startName << " and " << endName << ".\n";
        return;
    }

    const vector<int> path = result.buildPath(end);
    cout << "\nFastest Route Summary\n";
    cout << "Algorithm used: Traffic-Aware Dijkstra\n";
    cout << "Estimated travel time: " << result.distances[end] << " minutes\n";
    cout << "Route distance: " << pathWeight(distanceGraph, path) << " km\n";
    cout << "Accumulated traffic score: " << pathWeight(intensityGraph, path) << '\n';
    cout << "Route: ";
    printPath(distanceGraph, path);
    printSegmentBreakdown(distanceGraph, intensityGraph, path);
}

void handleNetworkSummary(const Graph& distanceGraph, const Graph& intensityGraph) {
    // This feature highlights broader graph properties instead of a single route query.
    const vector<vector<int>> allPairs = distanceGraph.floydWarshall();
    int blockedRoads = 0;
    int openRoads = 0;
    int unreachablePairs = 0;
    int busiestTraffic = -1;
    pair<int, int> busiestRoad = {-1, -1};
    int diameter = -1;
    pair<int, int> diameterPair = {-1, -1};
    int graphCenter = -1;
    double bestAverageDistance = numeric_limits<double>::max();
    int bestFarthestDistance = INF;

    for (int from = 0; from < distanceGraph.vertexCount(); ++from) {
        long long sumDistances = 0;
        int reachableCount = 0;
        int farthestDistance = 0;

        for (int to = 0; to < distanceGraph.vertexCount(); ++to) {
            if (from == to) {
                continue;
            }

            if (distanceGraph.hasEdge(from, to)) {
                ++openRoads;
            } else {
                ++blockedRoads;
            }

            if (intensityGraph.hasEdge(from, to) && intensityGraph.weight(from, to) > busiestTraffic) {
                busiestTraffic = intensityGraph.weight(from, to);
                busiestRoad = {from, to};
            }

            if (allPairs[from][to] >= INF) {
                ++unreachablePairs;
                continue;
            }

            sumDistances += allPairs[from][to];
            ++reachableCount;
            farthestDistance = max(farthestDistance, allPairs[from][to]);

            if (allPairs[from][to] > diameter) {
                diameter = allPairs[from][to];
                diameterPair = {from, to};
            }
        }

        if (reachableCount == 0) {
            continue;
        }

        // The node with the lowest average shortest-path distance acts as the graph center.
        const double averageDistance = static_cast<double>(sumDistances) / reachableCount;
        if (averageDistance < bestAverageDistance ||
            (averageDistance == bestAverageDistance && farthestDistance < bestFarthestDistance)) {
            bestAverageDistance = averageDistance;
            bestFarthestDistance = farthestDistance;
            graphCenter = from;
        }
    }

    cout << "\nNetwork Summary\n";
    cout << "Algorithm used: Floyd-Warshall all-pairs shortest path\n";
    cout << "Open roads: " << openRoads << '\n';
    cout << "Blocked roads: " << blockedRoads << '\n';
    cout << "Unreachable ordered pairs after detours: " << unreachablePairs << '\n';

    if (graphCenter != -1) {
        cout << fixed << setprecision(2);
        cout << "Most central intersection: " << distanceGraph.vertexName(graphCenter)
             << " | average shortest distance: " << bestAverageDistance << " km"
             << " | farthest reachable node: " << bestFarthestDistance << " km\n";
        cout.unsetf(ios::floatfield);
        cout << setprecision(6);
    }

    if (diameterPair.first != -1) {
        cout << "Network diameter: " << diameter << " km"
             << " between " << distanceGraph.vertexName(diameterPair.first)
             << " and " << distanceGraph.vertexName(diameterPair.second) << '\n';
    }

    if (busiestRoad.first != -1) {
        cout << "Busiest direct road: " << intensityGraph.vertexName(busiestRoad.first)
             << " -> " << intensityGraph.vertexName(busiestRoad.second)
             << " with traffic intensity " << busiestTraffic << '\n';
    }
}

int main() {
    try {
        // Separate graphs are loaded for distance and traffic, then combined into a travel-time graph.
        const Graph distanceGraph = Graph::fromCsv("Islamabad_Intersections_Distance_Int.csv");
        const Graph intensityGraph = Graph::fromCsv("Islamabad_Traffic_Intensity.csv");
        ensureCompatibleGraphs(distanceGraph, intensityGraph);
        const Graph travelTimeGraph = buildTravelTimeGraph(distanceGraph, intensityGraph);

        cout << "Digital Traffic Management System\n";
        cout << "Loaded " << distanceGraph.vertexCount() << " intersections from CSV files.\n";
        cout << "Data structures in use: adjacency matrix, adjacency list, hash map, and priority queue.\n";

        while (true) {
            printMenu();
            const int choice = readMenuChoice();

            switch (choice) {
                case 1:
                    handleVipRoute(distanceGraph);
                    break;
                case 2:
                    handleDirectTraffic(distanceGraph, intensityGraph);
                    break;
                case 3:
                    handleGreenSignalPlan(intensityGraph);
                    break;
                case 4:
                    handleFastestRoute(distanceGraph, intensityGraph, travelTimeGraph);
                    break;
                case 5:
                    handleNetworkSummary(distanceGraph, intensityGraph);
                    break;
                case 6:
                    cout << "Exiting program. Goodbye!\n";
                    return 0;
                default:
                    cout << "Invalid choice. Please try again.\n";
                    break;
            }
        }
    } catch (const exception& error) {
        cerr << "Fatal error: " << error.what() << '\n';
        return 1;
    }
}
