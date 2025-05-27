#include "../include/Graph.h"
#include <future>
#include <mutex>
#include <queue>
#include <map>

#define INF 0x3f3f3f3f

Graph::Graph(int V)
{
    this->V = V;
    adj = new list<pair<int, int>>[V + 1];
    for (int i = 0; i < V; ++i)
    {
        addEdge(V, i, 0);
    }
}

void Graph::addEdge(int u, int v, int w)
{
    adj[u].push_back(make_pair(v, w));
    edges.push_back({u, v, w});
}

vector<int> Graph::Dijkstra(int src) {
    vector<int> dist(V, INT_MAX);
    dist[src] = 0;

    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    pq.push({0, src});

    while (!pq.empty())
    {
        pair<int, int> top = pq.top();
        pq.pop();

        int u = top.second;
        int d = top.first;

        if (d > dist[u])
            continue;

        for (auto &edge : adj[u])
        {
            int v = edge.first;
            int weight = edge.second;

            if (dist[u] + weight < dist[v])
            {
                dist[v] = dist[u] + weight;
                pq.push({dist[v], v});
            }
        }
    }

    return dist;
}

vector<int> Graph::BellmanFord(int src)
{
    vector<int> dist(V + 1, INT_MAX);
    dist[src] = 0;

    for (int i = 1; i <= V; i++)
    {
        for (auto &edge : edges)
        {
            int u = get<0>(edge);
            int v = get<1>(edge);
            int w = get<2>(edge);

            if (u < dist.size() && v < dist.size() && dist[u] != INT_MAX && dist[u] + w < dist[v])
            {
                dist[v] = dist[u] + w;
            }
        }
    }

    for (auto &edge : edges)
    {
        int u = get<0>(edge);
        int v = get<1>(edge);
        int w = get<2>(edge);

        if (u < dist.size() && v < dist.size() && dist[u] != INT_MAX && dist[u] + w < dist[v])
        {
            cout << "Graph contains a negative weight cycle!" << endl;
            return {};
        }
    }

    return dist;
}

void Graph::PrintGraph()
{
    cout << "Graph:" << endl;
    for (int i = 0; i < V + 1; i++)
    {
        cout << "Vertex " << i << ": ";
        for (const auto &neighbor : adj[i])
        {
            cout << "(" << neighbor.first << ", weight=" << neighbor.second << ") ";
        }
        cout << endl;
    }
}

vector<vector<int>> Graph::Johnson(bool multiThreaded)
{
    vector<int> h = BellmanFord(V);
    if (h.empty())
        return {};

    for (auto &[u, v, w] : edges)
    {
        w += (h[u] - h[v]);
    }

    for (int u = 0; u < V; u++)
    {
        for (auto &edge : adj[u])
        {
            int v = edge.first;
            edge.second += (h[u] - h[v]);
        }
    }

    vector<vector<int>> dist(V, vector<int>(V, INF));
    if (multiThreaded)
        JohnsonMultiThreaded(dist);
    else
        JohnsonSingleThreaded(dist);

    for (int i = 0; i < V; i++)
    {
        for (int j = 0; j < V; j++)
        {
            if (dist[i][j] < INF)
            {
                dist[i][j] += (h[j] - h[i]);
            }
        }
    }

    for (auto &[u, v, w] : edges)
    {
        w += (-h[u] + h[v]);
    }

    for (int u = 0; u < V; u++)
    {
        for (auto &edge : adj[u])
        {
            int v = edge.first;
            edge.second += (-h[u] + h[v]);
        }
    }

    return dist;
}

void Graph::JohnsonSingleThreaded(vector<vector<int>> &dist)
{
    for (int i = 0; i < V; i++)
    {
        vector<int> d = Dijkstra(i);
        if (d.size() >= V)
        {
            for (int j = 0; j < V; j++)
            {
                dist[i][j] = d[j];
            }
        }
        else
        {
            cout << "Error: Dijkstra returned a vector of unexpected size for vertex " << i << endl;
        }
    }
}

void Graph::JohnsonMultiThreaded(vector<vector<int>> &dist)
{
    vector<future<vector<int>>> results(V);

    for (int i = 0; i < V; i++)
    {
        results[i] = async(launch::async, [this, i]()
                           { return Dijkstra(i); });
    }

    for (int i = 0; i < V; i++)
    {
        vector<int> d = results[i].get();

        if (d.size() >= V)
        {
            for (int j = 0; j < V; j++)
            {
                dist[i][j] = d[j];
            }
        }
        else
        {
            cout << "Error: Dijkstra returned a vector of unexpected size for vertex " << i << endl;
        }
    }
}
