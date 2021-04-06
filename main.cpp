#include <iostream>
#include <vector>
#include <queue>
#include <climits>
#include <fstream>
using namespace std;


struct Edge
{
    int source, dest, weight;
};

// Data structure to store a heap node
struct Node {
    int vertex, weight;
};

//
class Graph
{
public:
    // liste d'adjacence
    vector<vector<Edge>> adjList;


    Graph(vector<Edge> const &edges, int v)
    {
        // redimmensioner le vector pour avoir le nombre sommet
        adjList.resize(v);

        // ajoutez les arrete non orienté
        for (Edge const &edge: edges)
        {

            adjList[edge.source].push_back(edge);
        }
    }
};

void print_route(vector<int> const &prev, int i)
{


    if (i < 0)
        {
        return;
        }


    print_route(prev, prev[i]);
    cout << i << " ";

}


struct comp
{
    bool operator()(const Node &lhs, const Node &rhs) const {
        return lhs.weight > rhs.weight;
    }
};


void findShortestPaths(Graph const &graph, int source, int v, int fin)
{
    // prendre la source comme arrete = 0
    priority_queue<Node, vector<Node>, comp> min_heap;
    min_heap.push({source, 0});

    // mettre la distnace initial pour la source a l'infini
     vector<int> dist(v, INT_MAX);
    //distnace de la source a elle meme a 0
    dist[source] = 0;

    // tableau de boolean pour traquer les sommets pour trouver le chemin minimum poiur chaque chemin

    vector<bool> done(v, false);
    done[source] = true;

    // stocker les predecesseur pour ecrire le chemin
    vector<int> prev(v, -1);

    // run jusqu'a qu'au sommet final
    while (!min_heap.empty())
    {
        //  enlever et retourner la meilleurs sommet
        Node node = min_heap.top();
        min_heap.pop();

        // obtenir le numero du sommet
        int u = node.vertex;


        for (auto i: graph.adjList[u])
        {
            int v = i.dest;
            int weight = i.weight;


            if (!done[v] && (dist[u] + weight) < dist[v])
            {
                dist[v] = dist[u] + weight;
                prev[v] = u;
                min_heap.push({v, dist[v]});
            }
        }

        // marquage des sommet
        done[u] = true;
    }

    for (int i = 0; i < v; i++)
    {
        if (i != source && dist[i] != INT_MAX && i == fin)
        {
            cout << "Chemin (" << source << " --> " << i << "): Poids minimal = "
                 << dist[i] << ", Route = [ ";
            print_route(prev, i);
            cout << "]" << endl;

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
    vector<Edge> edges;

    while(flx)  /// on creer la liste d'adjacence grace au valeur récupérées dans le fichier
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
