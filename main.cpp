#include <iostream>
#include <vector>
#include <queue>
#include <climits>
#include <fstream>
#include "Graphe.h"
#include "Edge.h"
#include "Node.h"

void print_route(std::vector<int> const &prev, int i)
{
    if (i < 0)
        {
        return;
        }


    print_route(prev, prev[i]);
    std::cout << i << " ";
}


struct comp
{
    bool operator()(const Node &lhs, const Node &rhs) const {
        return lhs.getWeight() > rhs.getWeight();
    }
};


void findShortestPaths(Graph const &graph, int source, int v, int fin)
{
    // prendre la source comme arrete = 0
    std::priority_queue<Node, std::vector<Node>, comp> min_heap;
    min_heap.push({source, 0});

    // mettre la distnace initial pour la source a l'infini
     std::vector<int> dist(v, INT_MAX);
    //distnace de la source a elle meme a 0
    dist[source] = 0;

    // tableau de boolean pour traquer les sommets pour troiver le chemin minimum poiur chaque chemin

    std::vector<bool> done(v, false);
    done[source] = true;

    // stocker les predecesseur pour ecrire le chemin
    std::vector<int> prev(v, -1);

    // run qu'au sommet final
    while (!min_heap.empty())
    {
        //  enlever et retourner la meilleurs sommet
        Node node = min_heap.top();
        min_heap.pop();

        // obtenir le numero du sommet
        int u = node.getVertex();


        for (auto i: graph.m_adjList[u])
        {
            int v = i.getDest();
            int weight = i.getWeight();


            if (!done[v] && (dist[u] + weight) < dist[v])
            {
                dist[v] = dist[u] + weight;
                prev[v] = u;
                min_heap.push({v, dist[v]});
            }
        }

        // marquage des sommets
        done[u] = true;
    }

    for (int i = 0; i < v; i++)
    {
        if (i != source && dist[i] != INT_MAX && i == fin)
        {
            std::cout << "Chemin (" << source << " --> " << i << "): Poids minimal = "
                 << dist[i] << ", Route = [ ";
            print_route(prev, i);
            std::cout << "]" << std::endl;

        }
    }
}

int main()
{
    int v;
    std::ifstream flx("graph.txt");
    flx >> v;
    int cpt = 0;
    int v1;
    int v2;
    int poids;

    // initialize edges as per the above diagram
    std::vector<Edge> edges;

    while(flx)  /// on creer la liste d'adjacence grace au valeur r�cup�r�es dans le fichier
    {
        std::string ligne;
        getline(flx, ligne);

        if (cpt > v)
        {
            flx >> v1 >> v2 >> poids;
            edges.push_back({v1,v2,poids});
        }
        cpt++;

    }

    // construct graph
    Graph graph(edges, v);

    int source ;
    int fin ;
    std::cout <<" De quel sommet voulez vous partir ?"<<std::endl;
    std::cin >>source ;
    std::cout <<" Quel est le sommet d'arrive de votre choix ?"<<std::endl;
    std::cin>> fin ;
    findShortestPaths(graph, source, v , fin);

    return 0;
}
