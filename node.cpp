#include "Node.h"
#include <iostream>

Node::Node(int _vertex, int _weight) : m_vertex(_vertex), m_weight(_weight)
{
}

Node::getVertex() const
{
    return m_vertex;
}

Node::getWeight() const
{
    return m_weight;
}

Node::~Node()
{
}


