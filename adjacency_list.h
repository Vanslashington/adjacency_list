/*
    adjacency_list.h
    David Vaughan
    11/27/2014
*/

#ifndef ADJACENCY_LIST_H
#define ADJACENCY_LIST_H

#include <vector>
#include <map>

using namespace std;

// Adjacency List class for graph problems
template <typename dataType, typename weightType = int>
class adjacency_list
{
  public:
    // Constructors and destructor
    adjacency_list();
    adjacency_list(const adjacency_list& other);
    ~adjacency_list();

    // Assignment operator
    adjacency_list& operator=(adjacency_list other);

    // Other methods
    void reserve(int newCapacity);
    void insertVertex(const dataType& newVertex);
    void insertEdge(const dataType& vertexA, const dataType& vertexB);
    void insertEdge(const dataType& vertexA, const dataType& vertexB, 
                    const weightType& weight);
    void insertDirectedEdge(const dataType& vertexA, const dataType& vertexB);
    void insertDirectedEdge(const dataType& vertexA, const dataType& vertexB, 
                            const weightType& weight);

    // Shortest path
    weightType path(const dataType& vertexA, const dataType& vertexB);
    weightType path(const dataType& vertexA, const dataType& vertexB, 
                    vector<dataType>& pathVector);

  private:
    // Edge struct to store links and weights
    struct edge
    {
        edge(dataType _vertex, weightType _weight)
            : vertex(_vertex), weight(_weight) {}
        dataType vertex;
        weightType weight;
    };

    // 2D vector to store adjacent vertices
    vector<vector<edge> > vertices;

    // Map from dataType to vector indices
    map<dataType, int> indexMap;

    // Other members
    int capacity;
    int size;
    bool weighted;
    bool negativeWeights;

    // Helper functions
    weightType bfs(const dataType& vertexA, const dataType& vertexB);
    weightType bfs(const dataType& vertexA, const dataType& vertexB, 
                   vector<dataType>& pathVector);

    weightType dijkstra(const dataType& vertexA, const dataType& vertexB);
    weightType dijkstra(const dataType& vertexA, const dataType& vertexB, 
                        vector<dataType>& pathVector);

    weightType bellman_ford(const dataType& vertexA, const dataType& vertexB);
    weightType bellman_ford(const dataType& vertexA, const dataType& vertexB, 
                            vector<dataType>& pathVector);
};

// Implementation file
#include "adjacency_list.tpp"

#endif
