/*
    adjacency_list.tpp
    David Vaughan
    11/27/2014
*/

#include <queue>
#include <utility>
#include <algorithm>

template <typename dataType, typename weightType>
adjacency_list<dataType, weightType>::adjacency_list()
    : size(0), weighted(false), negativeWeights(false)
{
    capacity = vertices.capacity();
}

template <typename dataType, typename weightType>
adjacency_list<dataType, weightType>::
    adjacency_list(const adjacency_list& other)
    : vertices(other.vertices), indexMap(other.indexMap)
{
    size = vertices.size();
    capacity = vertices.capacity();
}

template <typename dataType, typename weightType>
adjacency_list<dataType, weightType>::
    ~adjacency_list()
{
}

template <typename dataType, typename weightType>
adjacency_list<dataType, weightType>& adjacency_list<dataType, weightType>::
    operator=(adjacency_list other)
{
    swap(vertices, other.vertices);
    swap(indexMap, other.indexMap);
    size = vertices.size();
    capacity = vertices.capacity();
}

template <typename dataType, typename weightType>
void adjacency_list<dataType, weightType>::reserve(int newCapacity)
{
    vertices.reserve(newCapacity);
    capacity = newCapacity;
}

template <typename dataType, typename weightType>
void adjacency_list<dataType, weightType>::
    insertVertex(const dataType& newVertex)
{
    indexMap[newVertex] = size++;
    vertices.push_back(vector<edge>());
}

template <typename dataType, typename weightType>
void adjacency_list<dataType, weightType>::
    insertEdge(const dataType& vertexA, const dataType& vertexB)
{
    insertEdge(vertexA, vertexB, 1);
}

template <typename dataType, typename weightType>
void adjacency_list<dataType, weightType>::
    insertEdge(const dataType& vertexA, const dataType& vertexB, 
               const weightType& weight)
{
    insertDirectedEdge(vertexA, vertexB, weight);
    insertDirectedEdge(vertexB, vertexA, weight);
}

template <typename dataType, typename weightType>
void adjacency_list<dataType, weightType>::
    insertDirectedEdge(const dataType& vertexA, const dataType& vertexB)
{
    insertDirectedEdge(vertexA, vertexB, 1);
}

template <typename dataType, typename weightType>
void adjacency_list<dataType, weightType>::
    insertDirectedEdge(const dataType& vertexA, const dataType& vertexB, 
                       const weightType& weight)
{
    if(!indexMap.count(vertexA)) insertVertex(vertexA);
    if(!indexMap.count(vertexB)) insertVertex(vertexB);

    if(weight != 1) weighted = true;
    if(weight < 0) negativeWeights = true;

    vertices[indexMap[vertexA]].push_back(edge(vertexB, weight));
}

template <typename dataType, typename weightType>
weightType adjacency_list<dataType, weightType>::
    path(const dataType& vertexA, const dataType& vertexB)
{
    if(negativeWeights) return bellman_ford(vertexA, vertexB);
    if(weighted) return dijkstra(vertexA, vertexB);
    return bfs(vertexA, vertexB);
}

template <typename dataType, typename weightType>
weightType adjacency_list<dataType, weightType>::
    path(const dataType& vertexA, const dataType& vertexB, 
         vector<dataType>& pathVector)
{
    if(negativeWeights) return bellman_ford(vertexA, vertexB, pathVector);
    if(weighted) return dijkstra(vertexA, vertexB, pathVector);
    return bfs(vertexA, vertexB, pathVector);
}

template <typename dataType, typename weightType>
weightType adjacency_list<dataType, weightType>::
    bfs(const dataType& vertexA, const dataType& vertexB)
{
    queue<pair<dataType, weightType> > bfsQueue;
    vector<bool> visited(size, false);
    bfsQueue.push(pair<dataType, weightType>(vertexA, 0));

    while(!bfsQueue.empty())
    {
        dataType vertex = bfsQueue.front().first;
        weightType pathWeight = bfsQueue.front().second;
        int index = indexMap[vertex];
        visited[index] = true;

        if(vertex == vertexB) return pathWeight;

        bfsQueue.pop();
        
        for(typename vector<edge>::iterator i = vertices[index].begin();
                                            i != vertices[index].end(); ++i)
        {
            if(visited[indexMap[i->vertex]])
                bfsQueue.push(pair<dataType, weightType>(i->vertex,
                                                         1 + pathWeight));
        }
    }
}

template <typename dataType, typename weightType>
weightType adjacency_list<dataType, weightType>::
    bfs(const dataType& vertexA, const dataType& vertexB, 
        vector<dataType>& pathVector)
{   
    queue<pair<dataType, weightType> > bfsQueue;
    vector<bool> visited(size, false);
    vector<dataType> parent(size);
    bfsQueue.push(pair<dataType, weightType>(vertexA, 0));

    while(!bfsQueue.empty())
    {
        dataType vertex = bfsQueue.front().first;
        weightType pathWeight = bfsQueue.front().second;
        int index = indexMap[vertex];
        visited[index] = true;


        if(vertex == vertexB) break;

        bfsQueue.pop();
        
        for(typename vector<edge>::iterator i = vertices[index].begin();
                                            i != vertices[index].end(); ++i)
            if(!visited[indexMap[i->vertex]])
            {
                bfsQueue.push(pair<dataType, weightType>(i->vertex,
                                                         1 + pathWeight));
                parent[indexMap[i->vertex]] = vertex;
            }
    }

    dataType vertex = vertexB;
    pathVector.push_back(vertexB);
    while(vertex != vertexA)
        pathVector.push_back(vertex = parent[indexMap[vertex]]); 
    reverse(pathVector.begin(), pathVector.end());
    return bfsQueue.front().second; 
}

template <typename dataType, typename weightType>
weightType adjacency_list<dataType, weightType>::
    dijkstra(const dataType& vertexA, const dataType& vertexB)
{
}

template <typename dataType, typename weightType>
weightType adjacency_list<dataType, weightType>::
    dijkstra(const dataType& vertexA, const dataType& vertexB, 
             vector<dataType>& pathVector)
{
}

template <typename dataType, typename weightType>
weightType adjacency_list<dataType, weightType>::
    bellman_ford(const dataType& vertexA, const dataType& vertexB)
{
}

template <typename dataType, typename weightType>
weightType adjacency_list<dataType, weightType>::
    bellman_ford(const dataType& vertexA, const dataType& vertexB, 
                 vector<dataType>& pathVector)
{
}

