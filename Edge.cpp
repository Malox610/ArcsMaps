#include "Edge.h"
#include "Node.h"
#include <iostream>

Edge::Edge(Node _source, Node _dest, int _weight): m_source(_source), m_dest(_dest), m_weight(_weight)
{}

int Edge::getNum() const
{
    return m_num;
}

Node Edge::getSource()const
{
    return m_source;
}

Node Edge::getDest()const
{
    return m_dest;
}

int Edge::getWeight()const
{
    return m_weight;
}

Edge::~Edge(){}

std::string Edge::getType() const
{
    return m_type;
}

std::string Edge::getNom() const
{
    return m_nom;
}

void Edge::setWeight()
{
    m_weight = abs(m_dest.getWeight()- m_source.getWeight());
}

void Edge::setType(std::string newType)
{
    m_type = newType;
}

void Edge::setNom(std::string newNom)
{
    m_nom = newNom;
}

void Edge::setNum(int newNum)
{
    m_num = newNum;
}


void Edge::afficher()
{

    std::cout << "num : " << getNum() << "  " << "name : " << getNom()<< "  " << "type : " << getType() << "  " << "start : " << getSource().getVertex()<< "  " << "end : " << getDest().getVertex() << "    weight : " << getWeight() <<std::endl;

}


