#ifndef NODE_H_INCLUDED
#define NODE_H_INCLUDED
#include "string"

class Node
{
private :

    int m_vertex;
    int m_weight;
    std::string m_name;


public :

    Node(int _vertex,std::string _name, int _weight);
    Node();
    int getVertex() const;
    int getWeight()const;
    std::string getName()const;
    void setVertex(int newVertex);
    void setWeight(int newWeight);
    void setName(std::string newName);
    void afficher();
    ~Node();

};

#endif // NODE_H_INCLUDED
