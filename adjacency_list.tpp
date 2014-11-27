#include <iostream>
using namespace std;

template <typename dataType, typename weightType>
adjacency_list<dataType, weightType>::adjacency_list()
{
}

template <typename dataType, typename weightType>
adjacency_list<dataType, weightType>::adjacency_list(const adjacency_list& other)
{
}

template <typename dataType, typename weightType>
adjacency_list<dataType, weightType>::~adjacency_list()
{
}

template <typename dataType, typename weightType>
adjacency_list<dataType, weightType>& adjacency_list<dataType, weightType>::operator=(adjacency_list other)
{
}

template <typename dataType, typename weightType>
void adjacency_list<dataType, weightType>::reserve(int newCapacity)
{
}

template <typename dataType, typename weightType>
void adjacency_list<dataType, weightType>::insertVertex(const dataType& newVertex)
{
}

template <typename dataType, typename weightType>
void adjacency_list<dataType, weightType>::insertEdge(const dataType& vertexA, const dataType& vertexB)
{
}

template <typename dataType, typename weightType>
void adjacency_list<dataType, weightType>::insertEdge(const dataType& vertexA, const dataType& vertexB, const weightType& weight)
{
}

template <typename dataType, typename weightType>
void adjacency_list<dataType, weightType>::insertDirectedEdge(const dataType& vertexA, const dataType& vertexB)
{
}

template <typename dataType, typename weightType>
void adjacency_list<dataType, weightType>::insertDirectedEdge(const dataType& vertexA, const dataType& vertexB, const weightType& weight)
{
}

template <typename dataType, typename weightType>
weightType adjacency_list<dataType, weightType>::path(const dataType& vertexA, const dataType& vertexB)
{
}

template <typename dataType, typename weightType>
weightType adjacency_list<dataType, weightType>::path(const dataType& vertexA, const dataType& vertexB, vector<dataType>& pathVector)
{
}

template <typename dataType, typename weightType>
weightType adjacency_list<dataType, weightType>::bfs(const dataType& vertexA, const dataType& vertexB)
{
}

template <typename dataType, typename weightType>
weightType adjacency_list<dataType, weightType>::bfs(const dataType& vertexA, const dataType& vertexB, vector<dataType>& pathVector)
{
}

template <typename dataType, typename weightType>
weightType adjacency_list<dataType, weightType>::dijkstra(const dataType& vertexA, const dataType& vertexB)
{
}

template <typename dataType, typename weightType>
weightType adjacency_list<dataType, weightType>::dijkstra(const dataType& vertexA, const dataType& vertexB, vector<dataType>& pathVector)
{
}

template <typename dataType, typename weightType>
weightType adjacency_list<dataType, weightType>::bellman_ford(const dataType& vertexA, const dataType& vertexB)
{
}

template <typename dataType, typename weightType>
weightType adjacency_list<dataType, weightType>::bellman_ford(const dataType& vertexA, const dataType& vertexB, vector<dataType>& pathVector)
{
}

