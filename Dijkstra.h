//Djikstra's algorithm implementation borrowed from https://www.geeksforgeeks.org/printing-paths-dijkstras-shortest-path-algorithm/

#pragma once
//#include <bits/stdc++.h>
#include <vector>
#include <iostream>
using namespace std;
// A Java program for Dijkstra's
// single source shortest path
// algorithm. The program is for
// adjacency matrix representation
// of the graph.

int NO_PARENT = -1;

// Function to print shortest path
// from source to currentVertex
// using parents array
void printPath(int currentVertex, vector<int> parents, vector<int>& path)
{

    // Base case : Source node has
    // been processed
    if (currentVertex == NO_PARENT) {
        return;
    }
    printPath(parents[currentVertex], parents, path);
    path.push_back(currentVertex);
}


// Function that implements Dijkstra's
// single source shortest path
// algorithm for a graph represented
// using adjacency matrix
// representation

vector<vector<int>> dijkstra(vector<vector<int> > adjacencyMatrix, int startVertex)
{
    int nVertices = adjacencyMatrix[0].size();

    // shortestDistances[i] will hold the
    // shortest distance from src to i
    vector<int> shortestDistances(nVertices);

    // added[i] will true if vertex i is
    // included / in shortest path tree
    // or shortest distance from src to
    // i is finalized
    vector<bool> added(nVertices);

    // Initialize all distances as
    // INFINITE and added[] as false
    for (int vertexIndex = 0; vertexIndex < nVertices;
        vertexIndex++) {
        shortestDistances[vertexIndex] = INT_MAX;
        added[vertexIndex] = false;
    }

    // Distance of source vertex from
    // itself is always 0
    shortestDistances[startVertex] = 0;

    // Parent array to store shortest
    // path tree
    vector<int> parents(nVertices);

    // The starting vertex does not
    // have a parent
    parents[startVertex] = NO_PARENT;

    // Find shortest path for all
    // vertices
    for (int i = 1; i < nVertices; i++) {

        // Pick the minimum distance vertex
        // from the set of vertices not yet
        // processed. nearestVertex is
        // always equal to startNode in
        // first iteration.
        int nearestVertex = -1;
        int shortestDistance = INT_MAX;
        for (int vertexIndex = 0; vertexIndex < nVertices; vertexIndex++) 
        {
            if (!added[vertexIndex] && shortestDistances[vertexIndex] < shortestDistance) 
            {
                nearestVertex = vertexIndex;
                shortestDistance = shortestDistances[vertexIndex];
            }
        }

        // Mark the picked vertex as
        // processed
        added[nearestVertex] = true;

        // Update dist value of the
        // adjacent vertices of the
        // picked vertex.
        for (int vertexIndex = 0; vertexIndex < nVertices;
            vertexIndex++) {
            int edgeDistance
                = adjacencyMatrix[nearestVertex]
                [vertexIndex];

            if (edgeDistance > 0
                && ((shortestDistance + edgeDistance)
                    < shortestDistances[vertexIndex])) {
                parents[vertexIndex] = nearestVertex;
                shortestDistances[vertexIndex]
                    = shortestDistance + edgeDistance;
            }
        }
    }

    vector<vector<int>> paths;
    for (int vertexIndex = 0; vertexIndex < shortestDistances.size(); vertexIndex++)
    {
        if (vertexIndex != startVertex) 
        {
            vector<int> path;
            path.push_back(startVertex);
            path.push_back(shortestDistances[vertexIndex]);
            printPath(vertexIndex, parents, path);

            paths.push_back(path);
        }
    }

    return paths;

}