#include "Edge.h"
#include <iostream>

Edge::Edge(int _source, int _dest, int _weight, std::string _type): m_source(_source), m_dest(_dest), m_weight(_weight), m_type(_type)
{}

int Edge::getSource()const
{
    return m_source;
}

int Edge::getDest()const
{
    return m_dest;
}

int Edge::getWeight()const
{
    return m_weight;
}

Edge::~Edge(){}
