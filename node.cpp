#include "Node.h"
#include <iostream>
#include "string"
Node::Node(int _vertex, std::string _name, int _weight) : m_vertex(_vertex), m_name(_name), m_weight(_weight)
{
}

int Node::getVertex() const
{
    return m_vertex;
}

int Node::getWeight() const
{
    return m_weight;
}

std::string Node::getName()const
{
    return m_name;
}

Node::~Node()
{
}

Node::Node()
{

}

void Node::setVertex(int newVertex)
{
    m_vertex = newVertex;
}

void Node::setWeight(int newWeight)
{
    m_weight = newWeight;
}

void Node::setName(std::string newName)
{
    m_name = newName;
}

void Node::afficher()
{
    std::cout << "num  : " << getVertex()<< " ";
    std::cout << "name : " << getName()<< "   ";
    std::cout << "alt  : " << getWeight() << std::endl;
}






