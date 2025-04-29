#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <climits>
#include <algorithm>
#include <queue>

using namespace std;

class Graph
{
private:
    unordered_map<string, vector<pair<string, int>>> adj;

public:
    void displayPathAndCost(string source, string destination, unordered_map<string, int> &dist, unordered_map<string, string> &path)
    {
        if (dist[destination] == INT_MAX)
        {
            cout << "No path found from " << source << " to " << destination << endl;
            return;
        }

        vector<string> p;
        string c = destination;
        while (c != "-1")
        {
            p.push_back(c);
            c = path[c];
        }

        reverse(p.begin(), p.end());

        cout << "Distance:" << dist[destination] << endl
             << "Path:" << endl;
        int len = p.size();
        for (int i = 1; i < len; i++)
        {
            cout << p[i - 1] << " to " << p[i] << ": " << (dist[p[i]] - dist[p[i - 1]]) << endl;
        }
        cout << endl;
    }
    void dfsHelper(string source, string destination)
    {
        unordered_map<string, string> path;
        unordered_map<string, int> dist;
        for (auto &p : adj)
            dist[p.first] = INT_MAX;
        dist[source] = 0;
        path[source] = "-1";
        unordered_map<string, int> vis;
        for (auto &p : adj)
        {
            vis[p.first] = 0;
        }
        vis[source] = 1;
        dfs(destination, adj, source, 0, dist, path, vis);
        displayPathAndCost(source, destination, dist, path);
    }
    bool dfs(string destination, unordered_map<string, vector<pair<string, int>>> &adj, string city, int cost,
             unordered_map<string, int> &dist, unordered_map<string, string> &path, unordered_map<string, int> &vis)
    {
        if (city == destination)
            return true;
        for (auto e : adj[city])
        {
            string dCity = e.first;
            int c = e.second;
            if (vis[dCity] == 0)
            {
                dist[dCity] = c + cost;
                path[dCity] = city;
                vis[dCity] = 1;
                if (dfs(destination, adj, dCity, dist[dCity], dist, path, vis))
                    return true;
            }
        }
        return false;
    }
    void bfs(string source, string destination)
    {

        unordered_map<string, string> path;
        unordered_map<string, int> vis;
        for (auto &p : adj)
        {
            vis[p.first] = 0;
        }
        unordered_map<string, int> dist;
        for (auto &p : adj)
        {
            dist[p.first] = INT_MAX;
        }

        queue<pair<int, string>> q;
        q.push({0, source});

        dist[source] = 0;
        vis[source] = 1;
        path[source] = "-1";

        while (!q.empty())
        {
            string sourceCity = q.front().second;
            int pathCost = q.front().first;
            q.pop();

            if (sourceCity == destination)
                break;

            for (auto e : adj[sourceCity])
            {
                string destCity = e.first;
                int cost = e.second;
                if (vis[destCity] != 1)
                {
                    vis[destCity] = 1;
                    path[destCity] = sourceCity;
                    dist[destCity] = cost + pathCost;
                    q.push({dist[destCity], destCity});
                }
            }
        }

        displayPathAndCost(source, destination, dist, path);
    }

    void ucs(string source, string destination)
    {

        unordered_map<string, string> path;
        unordered_map<string, int> dist;
        for (auto &p : adj)
        {
            dist[p.first] = INT_MAX;
        }

        priority_queue<pair<int, string>, vector<pair<int, string>>, greater<pair<int, string>>> minHeap;
        minHeap.push({0, source});

        dist[source] = 0;
        path[source] = "-1";

        while (!minHeap.empty())
        {
            string sourceCity = minHeap.top().second;
            int pathCost = minHeap.top().first;
            minHeap.pop();
            for (auto e : adj[sourceCity])
            {
                string destCity = e.first;
                int cost = e.second;
                if (pathCost + cost < dist[destCity])
                {
                    dist[destCity] = pathCost + cost;
                    path[destCity] = sourceCity;
                    minHeap.push({dist[destCity], destCity});
                }
            }
        }

        displayPathAndCost(source, destination, dist, path);
    }

    void astarHelper(int args, string source, string destination, string heuristicFile)
    {
        unordered_map<string, int> heuristic;
        readHeuristic(heuristicFile, heuristic);
        astar(source, destination, heuristic);
    }

    void astar(string source, string destination, unordered_map<string, int> &heuristic)
    {
        priority_queue<pair<int, string>, vector<pair<int, string>>, greater<>> pq;
        unordered_map<string, string> parent;
        unordered_map<string, int> gCost;
        for (auto cityInfo : adj)
            gCost[cityInfo.first] = INT_MAX;

        pq.push({heuristic[source], source});
        gCost[source] = 0;
        parent[source] = "-1";
        // fCost = gCost[city] + heuristic[city]
        while (!pq.empty())
        {
            int fCost = pq.top().first;
            string currentCity = pq.top().second;
            pq.pop();
            int gCostCurrent = gCost[currentCity];

            if (currentCity == destination)
                break;

            for (auto neighbour : adj[currentCity])
            {
                string adjCity = neighbour.first;
                int edgeCost = neighbour.second;

                if (gCostCurrent + edgeCost < gCost[adjCity])
                {
                    gCost[adjCity] = gCostCurrent + edgeCost;
                    parent[adjCity] = currentCity;
                    int newFCost = gCost[adjCity] + heuristic[adjCity];
                    pq.push({newFCost, adjCity});
                }
            }
        }

        displayPathAndCost(source, destination, gCost, parent);
    }

    void readHeuristic(string filename, unordered_map<string, int> &heuristic)
    {
        ifstream infile(filename);
        string city;
        int h;
        while (infile >> city >> h)
        {
            heuristic[city] = h;
        }
        infile.close();
    }
    bool readInputFile(string inputFileName)
    {
        ifstream inputFile(inputFileName);

        if (!inputFile.is_open())
        {
            cout << "Failed to load and open file" << endl;
            return false;
        }

        string city1, city2;
        int pathCost;
        string line;
        while (getline(inputFile, line) && line != "END")
        {
            if (line.empty())
                continue;
            stringstream ss(line);

            ss >> city1 >> city2 >> pathCost;

            adj[city1].push_back({city2, pathCost});
            adj[city2].push_back({city1, pathCost});
        }
        inputFile.close();
        return true;
    }
};

int main(int args, char *argv[])
{

    if (args < 5)
    {
        cout << "Command should be in correct format with required fields" << endl;
        cout << "Format: <filename> <seach algo(bfs,dfs,ucs,a*)> <soucrce> <destination>";
        return 1;
    }
    Graph graph;
    string inputFileName = argv[1];

    if (!graph.readInputFile(inputFileName))
        return 1;

    string searchAlgo = argv[2];
    string source = argv[3];
    string destination = argv[4];

    if (searchAlgo == "ucs")
    {
        graph.ucs(source, destination);
    }
    else if (searchAlgo == "dfs")
    {
        graph.dfsHelper(source, destination);
    }
    else if (searchAlgo == "bfs")
    {
        graph.bfs(source, destination);
    }
    else if (searchAlgo == "a*")
    {
        if (args < 6)
        {
            cout << "Make sure Heuristic file is specified for astar search\n";
            cout << "Format: <filename> <seach algo(bfs,dfs,ucs,a*)> <soucrce> <destination> <heuristic file>";
            return false;
        }
        string heuristicFile = argv[5];
        graph.astarHelper(args, source, destination, heuristicFile);
    }
    else
    {
        cout << "Unknown Seach algorithm" << endl;
        return 1;
    }
}