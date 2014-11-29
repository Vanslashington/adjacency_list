/*
    adjacency_list.tpp
    David Vaughan
    11/27/2014
*/

#include <queue>
#include <utility>
#include <algorithm>

// Default constructor
template <typename dataType, typename weightType>
adjacency_list<dataType, weightType>::adjacency_list()
    : size(0), weighted(false), negativeWeights(false)
{
    capacity = vertices.capacity();
}

// Copy constructor
template <typename dataType, typename weightType>
adjacency_list<dataType, weightType>::
    adjacency_list(const adjacency_list& other)
    : vertices(other.vertices), indexMap(other.indexMap)
{
    size = vertices.size();
    capacity = vertices.capacity();
}

// Destructor
template <typename dataType, typename weightType>
adjacency_list<dataType, weightType>::
    ~adjacency_list()
{
    // Nothing needed here right now
}

// Assignment operator
template <typename dataType, typename weightType>
adjacency_list<dataType, weightType>& adjacency_list<dataType, weightType>::
    operator=(adjacency_list other)
{
    swap(vertices, other.vertices);
    swap(indexMap, other.indexMap);
    size = vertices.size();
    capacity = vertices.capacity();
}

// Reserve space for vertices
template <typename dataType, typename weightType>
void adjacency_list<dataType, weightType>::
    reserve(int newCapacity)
{
    // Just call the vector's reserve function
    vertices.reserve(newCapacity);
    capacity = newCapacity;
}

// Insert vertex
template <typename dataType, typename weightType>
void adjacency_list<dataType, weightType>::
    insertVertex(const dataType& newVertex)
{
    // Add a new vertex to the map and the vector
    indexMap[newVertex] = size++;
    vertices.push_back(vector<edge>());
}

// Insert undirected, unweighted edge between to vertices
template <typename dataType, typename weightType>
void adjacency_list<dataType, weightType>::
    insertEdge(const dataType& vertexA, const dataType& vertexB)
{
    // Call the weighted overload of insert edge with weight 1
    insertEdge(vertexA, vertexB, 1);
}

// Insert undirected, weighted edge
template <typename dataType, typename weightType>
void adjacency_list<dataType, weightType>::
    insertEdge(const dataType& vertexA, const dataType& vertexB, 
               const weightType& weight)
{
    // Call the directed, weighted insert function from each vertex to the other
    insertDirectedEdge(vertexA, vertexB, weight);
    insertDirectedEdge(vertexB, vertexA, weight);
}

// Insert directed, unweighted edge
template <typename dataType, typename weightType>
void adjacency_list<dataType, weightType>::
    insertDirectedEdge(const dataType& vertexA, const dataType& vertexB)
{
    // Call the directed, weighted insert function, with weight 1
    insertDirectedEdge(vertexA, vertexB, 1);
}

// Insert directed, weighted edge
template <typename dataType, typename weightType>
void adjacency_list<dataType, weightType>::
    insertDirectedEdge(const dataType& vertexA, const dataType& vertexB, 
                       const weightType& weight)
{
    // Insert the vertices if they don't yet exist
    if(!indexMap.count(vertexA)) insertVertex(vertexA);
    if(!indexMap.count(vertexB)) insertVertex(vertexB);

    // Update "weighted" and "negativeWeights" flags
    if(weight != 1) weighted = true;
    if(weight < 0) negativeWeights = true;

    // Push the edge into adjacency list vector
    vertices[indexMap[vertexA]].push_back(edge(vertexB, weight));
}

// Return the length of the shortest path between two vertices
template <typename dataType, typename weightType>
weightType adjacency_list<dataType, weightType>::
    path(const dataType& vertexA, const dataType& vertexB)
{
    // Depending on the type of graph, choose a path algorithm
    if(negativeWeights) return bellman_ford(vertexA, vertexB);
    if(weighted) return dijkstra(vertexA, vertexB);
    return bfs(vertexA, vertexB);
}

// Find and record the shortest path between two vertices
template <typename dataType, typename weightType>
weightType adjacency_list<dataType, weightType>::
    path(const dataType& vertexA, const dataType& vertexB, 
         vector<dataType>& pathVector)
{
    // Depending on the type of graph, choose a path algorithm
    if(negativeWeights) return bellman_ford(vertexA, vertexB, pathVector);
    if(weighted) return dijkstra(vertexA, vertexB, pathVector);
    return bfs(vertexA, vertexB, pathVector);
}

// Bredth-First Search
template <typename dataType, typename weightType>
weightType adjacency_list<dataType, weightType>::
    bfs(const dataType& vertexA, const dataType& vertexB)
{
    // Initialize the queue and 'visited' vector
    queue<pair<dataType, weightType> > bfsQueue;
    vector<bool> visited(size, false);
    bfsQueue.push(pair<dataType, weightType>(vertexA, 0));
    visited[indexMap[vertexA]] = true;

    // While the queue isn't empty
    while(!bfsQueue.empty())
    {
        dataType vertex = bfsQueue.front().first;
        weightType pathWeight = bfsQueue.front().second;
        int index = indexMap[vertex];

        // Check if we've found the end
        if(vertex == vertexB) return pathWeight;
        bfsQueue.pop();
        
        // Add all unvisited edges to the queue
        for(typename vector<edge>::iterator i = vertices[index].begin();
                                            i != vertices[index].end(); ++i)
            if(visited[indexMap[i->vertex]])
            {
                bfsQueue.push(pair<dataType, weightType>(i->vertex,
                                                         1 + pathWeight));
                // Update visited status
                visited[indexMap[i->vertex]] = true;
            }
    }
}

// Bredth-First Search with path recording
template <typename dataType, typename weightType>
weightType adjacency_list<dataType, weightType>::
    bfs(const dataType& vertexA, const dataType& vertexB, 
        vector<dataType>& pathVector)
{   
    // Initialize queue, 'visited' vector, and parent vector
    queue<pair<dataType, weightType> > bfsQueue;
    vector<bool> visited(size, false);
    vector<dataType> parent(size);
    bfsQueue.push(pair<dataType, weightType>(vertexA, 0));
    visited[indexMap[vertexA]] = true;

    // While the queue isn't empty
    while(!bfsQueue.empty())
    {
        dataType vertex = bfsQueue.front().first;
        weightType pathWeight = bfsQueue.front().second;
        int index = indexMap[vertex];

        // Check if we've found the end
        if(vertex == vertexB) break;
        bfsQueue.pop();
        
        // Add all unvisited edges to the queue
        for(typename vector<edge>::iterator i = vertices[index].begin();
                                            i != vertices[index].end(); ++i)
            if(!visited[indexMap[i->vertex]])
            {
                bfsQueue.push(pair<dataType, weightType>(i->vertex,
                                                         1 + pathWeight));
                // Keep track of each vertex's parent vertex
                parent[indexMap[i->vertex]] = vertex;

                // Update visited status
                visited[indexMap[i->vertex]] = true;
            }
    }

    // Iterating backwards with the parent vector, push vertices into the path
    dataType vertex = vertexB;
    pathVector.push_back(vertexB);
    while(vertex != vertexA)
        pathVector.push_back(vertex = parent[indexMap[vertex]]); 

    // Reverse the path so that it's start-to-end
    reverse(pathVector.begin(), pathVector.end());

    // Return the length
    return bfsQueue.front().second; 
}

// Dijkstra's SSSP algorithm
template <typename dataType, typename weightType>
weightType adjacency_list<dataType, weightType>::
    dijkstra(const dataType& vertexA, const dataType& vertexB)
{
}

// Dijkstra's with path recording
template <typename dataType, typename weightType>
weightType adjacency_list<dataType, weightType>::
    dijkstra(const dataType& vertexA, const dataType& vertexB, 
             vector<dataType>& pathVector)
{
}

// Bellman-Ford's SSSP algorithm
template <typename dataType, typename weightType>
weightType adjacency_list<dataType, weightType>::
    bellman_ford(const dataType& vertexA, const dataType& vertexB)
{
}

// Bellman-Ford's with path recording
template <typename dataType, typename weightType>
weightType adjacency_list<dataType, weightType>::
    bellman_ford(const dataType& vertexA, const dataType& vertexB, 
                 vector<dataType>& pathVector)
{
}

