#ifndef EDGE_H_INCLUDED
#define EDGE_H_INCLUDED
#include "string"
#include "Node.h"

class Edge{

private :

    int m_num;
    Node m_dest;
    int m_weight;
    Node m_source;
    std::string m_type;
    std::string m_nom;

public:



      Edge(Node _source, Node _dest, int _weight);

      int getNum() const;
      Node getSource ()const;
      Node getDest ()const;
      std::string getType()const;
      std::string getNom()const;
      int getWeight ()const;
      void setWeight();
      void setType(std::string newType);
      void setNom(std::string newNom);
      void setNum(int newNum);
      void afficher();
      ~Edge();

};


#endif // EDGE_H_INCLUDED
