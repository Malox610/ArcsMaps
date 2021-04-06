#include "Edge.h"
#include "Graphe.h"
#include <iostream>
#include <vector>

Graph::Graph(std::vector<Edge> const &edges, int v)
{
    // redimmensioner le vector pour avoir le nombre sommet
    m_adjList.resize(v);

    // ajoutez les arrete non orient�
    for (Edge const &edge: edges)
    {
        m_adjList[edge.getSource()].push_back(edge);
    }
}

Graph::~Graph()
{

}





