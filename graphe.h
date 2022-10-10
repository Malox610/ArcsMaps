#ifndef GRAPHE_H_INCLUDED
#define GRAPHE_H_INCLUDED
#include "Edge.h"
#include <iostream>
#include <vector>

class Graph
{
int v =37 ;
public :


    std::vector<std::vector<Edge>> m_adjList;
    Graph(std::vector<Edge> const &edges, int v);
    ~Graph();
};


#endif // GRAPHE_H_INCLUDED
