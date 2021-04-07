#include "Edge.h"
#include "Graphe.h"
#include <iostream>
#include <vector>

Graph::Graph(std::vector<Edge> const &edges, int v)
{
    // redimmensioner le vector pour avoir le nombre sommet
    m_adjList.resize(95);

    // ajoutez les arrete non orient�
    for (Edge const &edge: edges)
    {

        m_adjList[edge.getSource().getVertex()].push_back(edge);
        std::cout << edge.getSource().getVertex()<< std::endl;
    }
}

Graph::~Graph()
{

}





