#ifndef NODE_H_INCLUDED
#define NODE_H_INCLUDED

class Node
{
private :

    int m_vertex;
    int m_weight;


public :

    Node(int _vertex, int _weight);
    int getVertex() const;
    int getWeight()const;
    ~Node();

};

#endif // NODE_H_INCLUDED
