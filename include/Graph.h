#include <bits/stdc++.h>

using namespace std;
/**
 * @class Graph
 * @brief A class to represent a weighted directed graph and compute shortest paths using various algorithms.
 */

#ifndef GRAPH_H
#define GRAPH_H

class Graph
{
private:
    int V; ///< Number of vertices in the graph
    list<pair<int, int>> *adj; ///< Adjacency list (vertex, weight)
    vector<tuple<int, int, int>> edges; ///< List of edges (source, destination, weight)

    /**
     * @brief Helper function for Johnson's algorithm using a single thread.
     * @param dist A reference to a 2D vector that stores shortest path distances.
     */
    void JohnsonSingleThreaded(vector<vector<int>> &dist);

    /**
     * @brief Helper function for Johnson's algorithm using multithreading.
     * @param dist A reference to a 2D vector that stores shortest path distances.
     */
    void JohnsonMultiThreaded(vector<vector<int>> &dist);

public:
    /**
     * @brief Constructs a graph with the specified number of vertices.
     * @param V Number of vertices.
     */
    Graph(int V);

    /**
     * @brief Adds a directed edge from vertex u to vertex v with weight w.
     * @param u Source vertex.
     * @param v Destination vertex.
     * @param w Edge weight.
     */
    void addEdge(int u, int v, int w);

    /**
     * @brief Computes the shortest paths from the source vertex using Dijkstra's algorithm.
     * @param s Source vertex.
     * @return A vector of shortest distances from the source to all vertices.
     */
    std::vector<int> Dijkstra(int s);

    /**
     * @brief Computes shortest paths from a source vertex using Bellman-Ford algorithm.
     * @param src Source vertex.
     * @return A vector of shortest distances, or empty vector if negative cycle is detected.
     */
    vector<int> BellmanFord(int src);

    /**
     * @brief Computes all-pairs shortest paths using Johnson's algorithm.
     * @param multiThreaded If true, uses multithreading for Dijkstra steps.
     * @return A 2D vector of shortest path distances between all vertex pairs.
     */
    vector<vector<int>> Johnson(bool multiThreaded);

    /**
     * @brief Prints the adjacency list of the graph.
     */
    void PrintGraph();
};

#endif // GRAPH_H
