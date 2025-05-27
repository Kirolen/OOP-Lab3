#include "include/Graph.h"
#include <climits>
#include <random>
#include <chrono>

using namespace std;
using namespace std::chrono;
#define INF 0x3f3f3f3f
/**
 * @brief Benchmarks the execution time of Johnson's algorithm
 *        in both single-threaded and multi-threaded modes on a randomly generated graph.
 *
 * @param n Number of vertices in the graph.
 * @param density Probability (0.0 to 1.0) that an edge exists between any two nodes.
 *
 * This function creates a directed weighted graph with `n` vertices and adds
 * edges between nodes randomly based on the specified density.
 * It then runs Johnson's algorithm in both single-threaded and multi-threaded
 * modes, measuring and printing the execution time and speedup.
 */
void benchmark(int n, double density)
{
    cout << "Generating a graph with " << n << " vertices...\n";
    cout << endl;

    Graph g(n);

    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> weightDist(1, 20);
    uniform_real_distribution<> edgeProb(0, 1);

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            if (i != j && edgeProb(gen) < density)
            {
                g.addEdge(i, j, weightDist(gen));
            }
        }
    }

    cout << "Running the algorithm without multithreading...\n";
    auto start1 = high_resolution_clock::now();
    g.Johnson(false);
    auto stop1 = high_resolution_clock::now();
    auto duration1 = duration_cast<milliseconds>(stop1 - start1);
    cout << "Execution time: " << duration1.count() << " ms\n";
    cout << endl;
    cout << "Running the algorithm with multithreading...\n";
    auto start2 = high_resolution_clock::now();
    g.Johnson(true);
    auto stop2 = high_resolution_clock::now();
    auto duration2 = duration_cast<milliseconds>(stop2 - start2);
    cout << "Execution time: " << duration2.count() << " ms\n";
    cout << endl;
    cout << "Speedup: " << (double)duration1.count() / duration2.count() << " times faster\n";
}

/**
 * @brief Runs a series of unit tests to validate the correctness of the Johnson algorithm implementation.
 *
 * The function performs the following tests:
 * - An empty graph with 3 vertices and no edges.
 * - A graph with a single directed edge.
 * - A graph with negative edge weights but no negative weight cycles.
 * - A graph containing a negative weight cycle (should return an empty result).
 *
 * Each test validates specific expected outputs using assertions.
 * If all tests pass, a success message is printed.
 */
void runUnitTests()
{
    {
        Graph g(3);
        auto dist = g.Johnson(false);
        assert(dist.size() == 3);
        assert(dist[0][0] == 0);
        assert(dist[1][1] == 0);
        assert(dist[2][2] == 0);
    }

    {
        Graph g(3);
        g.addEdge(0, 1, 5);
        auto dist = g.Johnson(false);
        assert(dist[0][1] == 5);
        assert(dist[0][2] >= INF);
    }

    {
        Graph g(3);
        g.addEdge(0, 1, -2);
        g.addEdge(1, 2, -1);
        auto dist = g.Johnson(false);
        assert(dist[0][2] == -3);
    }

    {
        Graph g(3);
        g.addEdge(0, 1, 1);
        g.addEdge(1, 2, -2);
        g.addEdge(2, 0, -2);
        auto dist = g.Johnson(false);
        assert(dist.empty());
    }

    cout << "All unit tests passed!" << endl << endl;
}

int main()
{
    runUnitTests();
    benchmark(1000, 0.55);

    return 0;
}