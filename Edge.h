#ifndef EDGE_H_INCLUDED
#define EDGE_H_INCLUDED

class Edge{

private :

    int m_dest;
    int m_weight;
    int m_source;

public:



      Edge(int _source, int _dest, int _weight);
      int getSource ()const;
      int getDest ()const;
      int getWeight ()const;
      ~Edge();

};


#endif // EDGE_H_INCLUDED
