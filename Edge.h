#ifndef EDGE_H_INCLUDED
#define EDGE_H_INCLUDED
#include "string"
class Edge{

private :

    int m_dest;
    int m_weight;
    int m_source;
    std::string m_type;
    std::string m_nom;

public:



      Edge(int _source, int _dest, int _weight, std::string _type);
      int getSource ()const;
      int getDest ()const;
      int getWeight ()const;
      ~Edge();

};


#endif // EDGE_H_INCLUDED
