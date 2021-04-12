/*
    source Dijkstra : https://www.techiedelight.com/single-source-shortest-paths-dijkstras-algorithm/
    source BFS : https://www.softwaretestinghelp.com/cpp-bfs-program-to-traverse-graph/
*/
#include <iostream>
#include <vector>
#include <queue>
#include <climits>
#include <fstream>
#include <ctime>
#include <windows.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include "Graphe.h"
#include "Edge.h"
#include "Node.h"
#include "Skieur.h"



/// -----------------------------------------------------------------------------------------------------------------------------

void print_route(std::vector<int> const &prev, int i, std::vector<Edge> crossedEdge)
{
    HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
    if (i < 0)
    {
        return;
    }

    print_route(prev, prev[i], crossedEdge);
    int taille = crossedEdge.size();
    for(int x = 0; x < taille; x++)
    {
        if(prev[i]== crossedEdge[x].getSource().getVertex() && i == crossedEdge[x].getDest().getVertex())
        {
            std::string message="";
            char k = crossedEdge[x].getType();
            int mario =0; // nom au pif
            mario = k;
            switch(mario)
            { ///piste
                    case 78 : //piste noir
                         SetConsoleTextAttribute(hConsole, 240);
                   message="la piste noire ";
                    break ;

                    case 82 : //piste rouge
                       SetConsoleTextAttribute(hConsole, 244);
                   message="la piste rouge ";
                    break ;

                    case 66 : //piste bleu
                 SetConsoleTextAttribute(hConsole, 241);
                   message="la piste bleue ";
                    break ;

                    case 76 : //piste kilometre lance
                  SetConsoleTextAttribute(hConsole, 245);
                   message="la piste kilometre lance ";
                    break ;

                    case 70 : //snowpark
                  SetConsoleTextAttribute(hConsole, 250);
                   message="le snowpark " ;
                    break ;

                    ///remonte mecanique
                     case 75 : //teleski
                   SetConsoleTextAttribute(hConsole, 248);
                   message="le tire fesse ";
                    break ;

                      case 83 : //telesiege
                  SetConsoleTextAttribute(hConsole, 248);
                   message="le telesiege ";
                    break ;

                     case 68 : //telesiege debrayable
                   SetConsoleTextAttribute(hConsole, 248);
                   message="le telesiege debrayable ";
                    break ;

                     case 67 : //telecabine
                SetConsoleTextAttribute(hConsole, 248);
                   message="la telecabine ";
                    break ;

                     case 80 : //telepherique
                  SetConsoleTextAttribute(hConsole, 248);
                   message="le tepherique ";
                    break ;

                     case 85 : //Bus
                  SetConsoleTextAttribute(hConsole, 246);
                   message="la ";
                    break ;
            }
            std::cout << " -- " <<message<<crossedEdge[x].getNom()<< "--> ";
            SetConsoleTextAttribute(hConsole, 240);
        }
    }
    std::cout << i << " ";
}


struct comp
{
    bool operator()(const Node &lhs, const Node &rhs) const {
        return lhs.getWeight() > rhs.getWeight();
    }
};

std::vector<Edge> EdgesTxt(std::vector<Node> nodes)
{
    std::vector<Edge> edges;
    Node start;
    Node finish;
    int v1;
    int v2;
    int weight;
    int num;
    char edgeType;
    std::string edgeName;
    std::ifstream flxEdges("fichier/data_arcs.txt"); // ouverture du fichier arcs

    while(flxEdges)
    {
        flxEdges >> num >> edgeName >> edgeType >> v1 >> v2;
        start = nodes[v1-1];
        finish = nodes[v2-1];
        Edge edge(start, finish, weight);
        weight = abs(start.getWeight()- finish.getWeight());
        edge.setType(edgeType);

        char k = edge.getType();
                int mario =0; // nom au pif
                mario = k;
                switch(mario)
                { ///piste
                    case 78 : //piste noir
                    edge.setWeight((2*weight)/100);
                    break ;

                    case 82 : //piste rouge
                    edge.setWeight((3*weight)/100);
                    break ;

                    case 66 : //piste bleu
                    edge.setWeight((4*weight)/100);
                    break ;

                    case 76 : //piste kilpùetre lance
                    edge.setWeight((0.16*weight)/100);
                    break ;

                    case 70 : //snowpark
                    edge.setWeight((10*weight)/100);
                    break ;

                    ///remonte mecanique
                     case 75 : //teleski
                    edge.setWeight(((4*weight)/100)+1);
                    break ;

                      case 83 : //telesiege
                    edge.setWeight(((4*weight)/100)+1);
                    break ;

                     case 68 : //telesiege debrayable
                    edge.setWeight(((3*weight)/100)+1);
                    break ;

                     case 67 : //telecabine
                    edge.setWeight(((3*weight)/100)+2);
                    break ;

                     case 80 : //telepherique
                    edge.setWeight(((2*weight)/100)+4);
                    break ;

                }
        if((start.getName()=="arc1600"||start.getName()=="arc2000") && (finish.getName()=="arc1600"|| finish.getName()=="arc2000"))
        {
            edge.setWeight(40);
        }
        if((start.getName()=="arc1600"||start.getName()=="arc1800") && (finish.getName()=="arc1600"|| finish.getName()=="arcarc1800"))
        {
            edge.setWeight(30);
        }

        edge.setNom(edgeName);
        edge.setNum(num);
        edges.push_back(edge);



    }
    int taille = edges.size();

    for(int i = 0; i < taille; i++)
    {
        edges[i].afficher();
    }
    std::cout<<""<<std::endl;
return edges;
}


std::vector<Edge> EdgesTxtEx(std::vector<Node> nodes,Skieur s)
{
    std::vector<Edge> edges;
    Node start;
    Node finish;
    int v1;
    int v2;
    int weight;
    int num;
    char edgeType;
    std::string edgeName;
    char pisteEnleve=s.getParam2();
    std::ifstream flxEdges("fichier/data_arcs.txt"); // ouverture du fichier arcs

    while(flxEdges)
    {
        flxEdges >> num >> edgeName >> edgeType >> v1 >> v2;

        if(edgeType!=pisteEnleve)
        {
            start = nodes[v1-1];
            finish = nodes[v2-1];
            Edge edge(start, finish, weight);
            weight = abs(start.getWeight()- finish.getWeight());
            edge.setType(edgeType);

            char k = edge.getType();
                int mario =0; // nom au pif
                mario = k;
                switch(mario)
                { ///piste
                   case 78 : //piste noir
                    edge.setWeight((2*weight)/100);
                    break ;

                    case 82 : //piste rouge
                    edge.setWeight((3*weight)/100);
                    break ;

                    case 66 : //piste bleu
                    edge.setWeight((4*weight)/100);
                    break ;

                    case 76 : //piste kilpùetre lance
                    edge.setWeight((0.16*weight)/100);
                    break ;

                    case 70 : //snowpark
                    edge.setWeight((10*weight)/100);
                    break ;

                    ///remonte mecanique
                     case 75 : //teleski
                    edge.setWeight(((20000*weight)/100)+1);
                    break ;

                      case 83 : //telesiege
                    edge.setWeight(((20000*weight)/100)+1);
                    break ;

                     case 68 : //telesiege debrayable
                    edge.setWeight(((15000*weight)/100)+1);
                    break ;

                     case 67 : //telecabine
                    edge.setWeight(((15000*weight)/100)+2);
                    break ;

                     case 80 : //telepherique
                    edge.setWeight(((10000*weight)/100)+4);
                    break ;

                }
            if((start.getName()=="arc1600"||start.getName()=="arc2000") && (finish.getName()=="arc1600"|| finish.getName()=="arc2000"))
            {
                edge.setWeight(400);
            }
            if((start.getName()=="arc1600"||start.getName()=="arc1800") && (finish.getName()=="arc1600"|| finish.getName()=="arcarc1800"))
            {
                edge.setWeight(300);
            }

            edge.setNom(edgeName);
            edge.setNum(num);
            edges.push_back(edge);

        }


    }

    int taille = edges.size();
    for(int i = 0; i < taille; i++)
    {
        edges[i].afficher();
    }
    std::cout<<""<<std::endl;

return edges;
}


std::vector<Node> nodesTxt(int *v)
{
    std::vector<Node> nodes;
    int vertex;//
    int alt;//
    std::string nodeName;//
    std::ifstream flxPoints("fichier/data_points.txt"); // ouverture du fichier points

    flxPoints >> *v; // premiere ligne du texte  = nombre de sommet

    while(flxPoints)  /// on creer la liste d'adjacence grace au valeur r�cup�r�es dans le fichier
    {
        flxPoints >> vertex >> nodeName >> alt;
        Node sommet(vertex, nodeName, alt);
        nodes.push_back(sommet);
    }
    return nodes ;
}


void AllShortestPast(Graph const &graph, Node source, int v) /// Dijkstra tous les plus courts chemins
{
    // prendre la source comme arrete = 0
    std::priority_queue<Node, std::vector<Node>, comp> min_heap;
    min_heap.push({source.getVertex(), source.getName(), 0});
    std::vector<Node> nodes;
    int z=0;
    nodes = nodesTxt(&z);
    std::vector <Edge> crossedEdge;


    // mettre la distnace initial pour la source a l'infini
     std::vector<int> dist(v, INT_MAX);

    //distnace de la source a elle meme a 0
    dist[source.getVertex()] = 0;

    // tableau de boolean pour traquer les sommets pour troiver le chemin minimum poiur chaque chemin
    std::vector<bool> done(v, false);
    done[source.getVertex()] = true;

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
            int v = i.getDest().getVertex();
            int weight = i.getWeight();

            if (!done[v] && (dist[u] + weight) < dist[v])
            {
                dist[v] = dist[u] + weight;
                prev[v] = u;
                min_heap.push({v," v ", dist[v]});
                crossedEdge.push_back(i);
            }
        }

        // marquage des sommets
        done[u] = true;
    }

    for (int i = 1; i < v; i++)
    {
            ///calcul de l'heure d'arrive
            time_t actuel = time(0);
            tm *ltm = localtime(&actuel);
            int heure =ltm->tm_hour ;
            int minute =ltm->tm_min ;
            int tempsTrajet =dist[i];
            int minEstime =minute+tempsTrajet ;
            do
            {
                if(minEstime>=60)
                {
                    heure=heure+1;
                    minEstime=minEstime-60;
                }
                if(heure>=24)
                {
                    int diff =heure-24;
                    heure =diff ;
                }

            }while(minEstime>60 || heure>24);

///Affichage
            std::cout << std::endl;
            std::cout << std::endl;
            std::cout << "Chemin (" << source.getName() << " --> " << i << "): Temps estime = " << dist[i] <<" min"<< std::endl;
           if(minEstime<10)
           {
                std::cout << "  Heure d'arrive estimee a : " <<heure << "h0"<<  minEstime;
           }
           else
           {
                 std::cout << "  Heure d'arrive estimee a : " <<heure << "h"<<  minEstime;
           }
            std::cout << std::endl;
            std::cout << std::endl;
            std::cout << "           ------------------------------------------------------------" << std::endl;
            std::cout << std::endl;
            std::cout << "Route = [ ";
            print_route(prev, i, crossedEdge);
            std::cout << "]" << std::endl;
            std::cout << std::endl;
            std::cout << "           ------------------------------------------------------------" << std::endl;
            std::cout << std::endl;
            std::cout << std::endl;

    }

    std::cout<<" "<< std::endl ;
    std::cout<<" "<< std::endl ;
    int p=0;
    char c;
    do
    {
        std::cout <<" <----  retour menu ( ecrire 1)"<< std::endl;
        std::cin>>c;
        fflush(stdin);
        p=c;
    }while(p!=49);
    system("cls");

}


void findShortestPaths(Graph const &graph, Node source, int v, Node fin ) /// Dijkstra plus court chemins choix du depart + arrivee
{

    // prendre la source comme arrete = 0
    std::priority_queue<Node, std::vector<Node>, comp> min_heap;
    min_heap.push({source.getVertex(), source.getName(), 0});
    std::vector <Edge> crossedEdge;

    // mettre la distnace initial pour la source a l'infini
     std::vector<int> dist(v, INT_MAX);

    //distnace de la source a elle meme a 0
    dist[source.getVertex()] = 0;

    // tableau de boolean pour traquer les sommets pour trouver le chemin minimum pour chaque chemin
    std::vector<bool> done(v, false);
    done[source.getVertex()] = true;

    // stocker les predecesseurs pour ecrire le chemin
    std::vector<int> prev(v, -1);

    // run jusqu'au sommet final
    while (!min_heap.empty())
    {
        //  enlever et retourner la meilleurs sommet
        Node node = min_heap.top();
        min_heap.pop();

        // obtenir le numero du sommet
        int u = node.getVertex();

        for (auto i: graph.m_adjList[u])     /// <------------
        {
            int v = i.getDest().getVertex();
            int weight = i.getWeight();

            if (!done[v] && (dist[u] + weight) < dist[v])
            {
                dist[v] = dist[u] + weight;
                prev[v] = u;
                min_heap.push({v," v ", dist[v]});
                crossedEdge.push_back(i);
            }
        }

        // marquage des sommets
        done[u] = true;
    }

    for (int i = 0; i < v; i++)
    {
        if (i != source.getVertex() && dist[i] != INT_MAX && i == fin.getVertex())
        {

            ///calcul de l'heure d'arrive
            time_t actuel = time(0);
            tm *ltm = localtime(&actuel);
            int heure =ltm->tm_hour ;
            int minute =ltm->tm_min ;
            int tempsTrajet =dist[i];
            int minEstime =minute+tempsTrajet ;
            do
            {
                if(minEstime>=60)
                {
                    heure=heure+1;
                    minEstime=minEstime-60;
                }
                if(heure>=24)
                {
                    int diff =heure-24;
                    heure =diff ;
                }

            }while(minEstime>60 || heure>24);

            ///affichage
            std::cout << std::endl;
            std::cout << std::endl;
            std::cout << "Chemin (" << source.getName() << " --> " << i << "): Temps estime = " << dist[i] <<" min"<< std::endl;
            if(minEstime<10)
            {
                std::cout << "  Heure d'arrive estimee a : " <<heure << "h0"<<  minEstime;
            }
            else
            {
                std::cout << "  Heure d'arrive estimee a : " <<heure << "h"<<  minEstime;
            }
            std::cout << std::endl;
            std::cout << "           ------------------------------------------------------------" << std::endl;
            std::cout << std::endl;
            std::cout << "Route = [ ";
            print_route(prev, i, crossedEdge);
            std::cout << "]" << std::endl;
            std::cout << std::endl;
            std::cout << "           ------------------------------------------------------------" << std::endl;
            std::cout << std::endl;
            std::cout << std::endl;

        }

    }

    std::cout<<" "<< std::endl ;
    std::cout<<" "<< std::endl ;
    int p=0;
    char c;
    do
    {
        std::cout <<" <----  retour menu ( ecrire 1)"<< std::endl;
        std::cin>>c;
        fflush(stdin);
        p=c;

    }while(p!=49);
    system("cls");
}


// Avec une liste d'adjacence , faire un BFS a partir d'un sommetde depart jusqu'a un sommet de fin
void CheminBFS(std::vector< std::vector<Edge> > adjList, Node start,Node finish)
{
    std::cout << "Chemin : ";
    int n = adjList.size();

    // Cree un tableau de boolean pour les sommet deja visite pour garder une trace de quel sommet a ete visite
    bool visited[n] = { false };
    std::vector<int>ListeSommet;

    //  cree un une queue pour stocker les sommet
    std::queue<Node> q;

    //mettre en premier point de la queue le sommet de depart et le marque comme visite
    q.push(start);
    visited[start.getVertex()] = true;
    int fin = 0;

    // tant que la queue contient un sommmet et qu'elle n'a pas rencontre le sommet de fin
    while(q.empty() == false && fin!=1)
    {
        Node ver = q.front();
        int vertex=ver.getVertex();
        ListeSommet.push_back(vertex);
        q.pop();

        // boucle pour connaitre tout ses voisin
        int taille = adjList[vertex].size();
        for(int i = 0; i <taille ; i++)
        {

            Node neighbor = adjList[vertex][i].getDest();
            if (vertex!=finish.getVertex())
            {
                if(visited[neighbor.getVertex()] == false)
                {
                    q.push(neighbor);
                    visited[neighbor.getVertex()] = true;
                }
            }
            else
            {
                fin=1;
            }
        }
        std::cout << vertex << " ";
        }
        std::cout<<" "<< std::endl ;
        std::cout<<" "<< std::endl ;
        int p=0;
        char c;
        do
        {
            std::cout <<" <----  retour menu ( ecrire 1)"<< std::endl;
            std::cin>>c;
            fflush(stdin);
            p=c;
        }while(p!=49);
    system("cls");
}

// Avec une liste d'adjacence , faire un BFS a partir d'un sommet de depart
void AdjListBFS(std::vector< std::vector<Edge> > adjList, Node start)
{
    std::cout << "Tout les sommets ateignable dans l'ordre croissant : ";
    int n = adjList.size();

     // Cree un tableau de boolean pour les sommet deja visite pour garder une trace de quel sommet a ete visite
    bool visited[n] = { false };

     // cree un une queue pour stocker les sommet
    std::queue<Node> q;

    //mettre en premier point de la queue le sommet de depart et le marque comme visite
    q.push(start);
    visited[start.getVertex()] = true;

    // tant que la queue contient un sommmet
    while(q.empty() == false)
    {
        Node ver = q.front();
        int vertex=ver.getVertex();
        q.pop();

        std::cout << vertex << " ";

       // boucle pour connaitre tout ses voisin
        int taille = adjList[vertex].size();
        for(int i = 0; i <taille ; i++)
        {
           //Si le voisin d'a pas encore ete visite on le rentre dans la queue et on le marque comme visite
            Node neighbor = adjList[vertex][i].getDest();

            if(visited[neighbor.getVertex()] == false)
            {
                q.push(neighbor);
                visited[neighbor.getVertex()] = true;
            }
        }
    }

    std::cout<<" "<< std::endl ;
    std::cout<<" "<< std::endl ;

    int p=0;
    char c;
    do
    {
        std::cout <<" <----  retour menu ( ecrire 1)"<< std::endl;
        std::cin>>c;
        fflush(stdin);
        p=c;
    }while(p!=49);
    system("cls");
}


void TrouverLeCheminLePlusCourt()
{


    int v = 0 ;//
    std::string nodeName;//
    std::string edgeName;
    std::vector<Node> nodes;
    std::vector<Edge> edges;

    int saisieSource;
    int saisieFin;
    Node source ;
    Node fin ;


    nodes = nodesTxt(&v);
    edges = EdgesTxt(nodes);
    Graph graph(edges, 95);
    Graph graphDure(edges, 95);

    int choix ;
    char b ;

    do
    {
        std::cout<<""<<std::endl;
        std::cout<<""<<std::endl;
        std::cout<<"                                          normal               "<<std::endl;
        std::cout<<""<<std::endl;
        std::cout<<""<<std::endl;
        std::cout<<""<<std::endl;
        std::cout<<""<<std::endl;
        std::cout<<""<<std::endl;
        std::cout <<"         -------------------------                                            /%%%%%%"<<std::endl;
        std::cout <<"         -------------------------                                          %%%%.  #%%%&"<<std::endl;
        std::cout <<"       || Que voulez vous faire ?  ||                                    %%%%%        &%%%"<<std::endl ;
        std::cout <<"       ||                          ||                                  &%%%%&#*,.  .*(%&%%%%*" <<std::endl;
        std::cout <<"       ||   1. Dijkstra            ||                                &%%% /%            % *%%%*"<<std::endl;
        std::cout <<"       ||   2. Tout chemin         ||                              %%%%     &%        &(    /%%%"<<std::endl;
        std::cout <<"       ||   3. BFS                 ||                            .%%%.        %.    %&        &%%%"<<std::endl;
        std::cout <<"       ||   4. BFS 2 sommet        ||                           %%%%           *%  %            %%%&"<<std::endl;
        std::cout <<"       ||   5. Retour au menu      ||                         *%%%   ./%&&&%%%%%%%%%%%%%%&&&%(,  .%%%"<<std::endl;
        std::cout <<"         -------------------------                           &%%%              #%  %               %%%("<<std::endl;
        std::cout <<"         -------------------------                          %%%  %/           %     *%           %& #%%&"<<std::endl;
        std::cout <<"                                                          .%%&    ,%        %#        &&        %     %%%"<<std::endl;
        std::cout <<"                                                         *%%&       &(    %&            %     &&       %%%"<<std::endl;
        std::cout <<"                                                        *%%&         .%  &               *%  %          %%%"<<std::endl;
        std::cout <<"                                                       ,%%%      (&&%%%%&%%%%&&&&&&&&&&%%%%%%%%&@&*      %%%"<<std::endl;
        std::cout <<"                                                       %%%&          %& .%               ,%  %          &#%%%"<<std::endl;
        std::cout <<"                                                      %%%  %        %     &%            %/    (%        % (%%%"<<std::endl;
        std::cout <<"                                                      #%%#  &&     %#        %         /%        &%     %.  &%%"<<std::endl;
        std::cout <<"                                                      %%%    &/  %&           &&      %,           %   &(   .%%&"<<std::endl;
        std::cout <<"                                                      %%%%%%&,%,%               %   %&              (%&/#&%%%%%%"<<std::endl;
        std::cout <<"                                                           %%%%%%%%%%%%%&&%(*,.  &&%  ..,*#&&&%%%%%%%%%%%&#"<<std::endl;
        std::cout <<"                                                                     .*%&&%%%%%%%%%%%%%%%%%%&&#,"<<std::endl;


   do
   {
      std::cin>>b;
      fflush(stdin);
      choix =b ;
   }while(choix !=49 && choix !=50 && choix !=51 && choix !=52 && choix !=53);

    switch(choix)
    {
        case 49 :
        system("cls");
        do
        {
            do
            {
                std::cout <<" De quel sommet voulez vous partir ?"<<std::endl;
                std::cin >>saisieSource;
           }while(saisieSource<1);

        }while(saisieSource>37);
        do
        {
            do
            {
                std::cout <<" Quel est le sommet d'arrive de votre choix ?"<<std::endl;
                std::cin>> saisieFin;
            }while(saisieFin<1);
        }while(saisieFin>37);

        source = nodes[saisieSource-1];
        fin = nodes[saisieFin-1];

        findShortestPaths(graphDure,source, v , fin);

        break ;

        case 50:
        system("cls");

            do
            {
                do
                {
                    std::cout <<" De quel sommet voulez vous partir ?"<<std::endl;
                    std::cin >>saisieSource;
                }while(saisieSource<1);
            }while(saisieSource>37);
            source = nodes[saisieSource-1];

            AllShortestPast(graph, source, v);

        break ;

        case 51:
            system("cls");
            do
            {
                do
                {
                    std::cout <<" De quel sommet voulez vous partir ?"<<std::endl;
                    std::cin >>saisieSource;
                }while(saisieSource<1);
            }while(saisieSource>37);

            source = nodes[saisieSource-1];
            AdjListBFS(graph.m_adjList, source);
        break ;

        case 52 :
            system("cls");
            do
            {
                do
                {
                    std::cout <<" De quel sommet voulez vous partir ?"<<std::endl;
                    std::cin >>saisieSource;
                }while(saisieSource<1);
            }while(saisieSource>37);
            do
            {
                do
                {
                    std::cout <<" Quel est le sommet d'arrive de votre choix ?"<<std::endl;
                    std::cin>> saisieFin;
                }while(saisieFin<1);
            }while(saisieFin>37);

            source = nodes[saisieSource-1];
            fin = nodes[saisieFin-1];
            CheminBFS(graph.m_adjList, source,fin);
            break ;

    }
   system("cls");
   }while(choix !=53);

}

void TrouverLeCheminLePlusCourtSpecial(Skieur s)
{

    int v = 0 ;//
    std::string nodeName;//
    std::string edgeName;
    std::vector<Node> nodes;
    std::vector<Edge> edges;
    std::vector<Edge> edgesDure;

    int saisieSource;
    int saisieFin;
    Node source ;
    Node fin ;

    nodes = nodesTxt(&v);

    edges = EdgesTxt(nodes);
    edgesDure = EdgesTxtEx(nodes,s);

    Graph graph(edges, 95);
    Graph graphDure(edgesDure, 95);

    int choix ;
    char b ;

    do
    {
        std::cout<<std::endl;
        std::cout<<std::endl;
        std::cout<<"                                               special                 "<<std::endl;
        std::cout<<std::endl;
        std::cout<<std::endl;
        std::cout<<std::endl;
        std::cout<<std::endl;
        std::cout<<std::endl;
        std::cout <<"         -------------------------                                            /%%%%%%"<<std::endl;
        std::cout <<"         -------------------------                                          %%%%.  #%%%&"<<std::endl;
        std::cout <<"       || Que voulez vous faire ?  ||                                    %%%%%        &%%%"<<std::endl ;
        std::cout <<"       ||                          ||                                  &%%%%&#*,.  .*(%&%%%%*" <<std::endl;
        std::cout <<"       ||   1. Dijkstra            ||                                &%%% /%            % *%%%*"<<std::endl;
        std::cout <<"       ||   2. Tout chemin         ||                              %%%%     &%        &(    /%%%"<<std::endl;
        std::cout <<"       ||   3. BFS 2 sommet        ||                            .%%%.        %.    %&        &%%%"<<std::endl;
        std::cout <<"       ||   4. Retour au menu      ||                           %%%%           *%  %            %%%&"<<std::endl;
        std::cout <<"       ||                          ||                         *%%%   ./%&&&%%%%%%%%%%%%%%&&&%(,  .%%%"<<std::endl;
        std::cout <<"         -------------------------                           &%%%              #%  %               %%%("<<std::endl;
        std::cout <<"         -------------------------                          %%%  %/           %     *%           %& #%%&"<<std::endl;
        std::cout <<"                                                          .%%&    ,%        %#        &&        %     %%%"<<std::endl;
        std::cout <<"                                                         *%%&       &(    %&            %     &&       %%%"<<std::endl;
        std::cout <<"                                                        *%%&         .%  &               *%  %          %%%"<<std::endl;
        std::cout <<"                                                       ,%%%      (&&%%%%&%%%%&&&&&&&&&&%%%%%%%%&@&*      %%%"<<std::endl;
        std::cout <<"                                                       %%%&          %& .%               ,%  %          &#%%%"<<std::endl;
        std::cout <<"                                                      %%%  %        %     &%            %/    (%        % (%%%"<<std::endl;
        std::cout <<"                                                      #%%#  &&     %#        %         /%        &%     %.  &%%"<<std::endl;
        std::cout <<"                                                      %%%    &/  %&           &&      %,           %   &(   .%%&"<<std::endl;
        std::cout <<"                                                      %%%%%%&,%,%               %   %&              (%&/#&%%%%%%"<<std::endl;
        std::cout <<"                                                           %%%%%%%%%%%%%&&%(*,.  &&%  ..,*#&&&%%%%%%%%%%%&#"<<std::endl;
        std::cout <<"                                                            .*%&&%%%%%%%%%%%%%%%%%%&&#,"<<std::endl;
        do
        {
            std::cin>>b;
            fflush(stdin);
            choix =b ;
        }while(choix !=49 && choix !=50 && choix !=51 && choix !=52 );


        switch(choix)
        {
            case 49 :
            system("cls");

            do
            {
                do
                {
                    std::cout <<" De quel sommet voulez vous partir ?"<<std::endl;
                    std::cin >>saisieSource;
                }while(saisieSource<1);
            }while(saisieSource>37);

            do
            {
                do
                {
                    std::cout <<" Quel est le sommet d'arrive de votre choix ?"<<std::endl;
                    std::cin>> saisieFin;
                }while(saisieFin<1);
            }while(saisieFin>37);

            source = nodes[saisieSource-1];
            fin = nodes[saisieFin-1];

            findShortestPaths(graphDure,source, v , fin);
            break ;

            case 50:
            system("cls");
            do
            {
                do
                {
                    std::cout <<" De quel sommet voulez vous partir ?"<<std::endl;
                    std::cin >>saisieSource;
                }while(saisieSource<1);
            }while(saisieSource>37);

            source = nodes[saisieSource-1];
            AllShortestPast(graphDure, source, v);

            break ;

            case 51:    ///BFS d'un point a un autre
               system("cls");
            do
            {
                do
                {
                    std::cout <<" De quel sommet voulez vous partir ?"<<std::endl;
                    std::cin >>saisieSource;

                }while(saisieSource<1);
            }while(saisieSource>37);
            do
            {
                do
                {
                    std::cout <<" Quel est le sommet d'arrive de votre choix ?"<<std::endl;
                    std::cin>> saisieFin;
                }while(saisieFin<1);
                }while(saisieFin>37);

            source = nodes[saisieSource-1];
            fin = nodes[saisieFin-1];

            CheminBFS(graph.m_adjList, source,fin);

            break ;

        }
        system("cls");

   }while(choix !=52);

}

int menu ()
{
    int choix ;
    char b ;
    Skieur s;

    std::cout<<""<<std::endl;
    do
    {
        std::cout<<""<<std::endl;
        std::cout<<std::endl;
        std::cout<<std::endl;
        std::cout<<std::endl;
        std::cout<<std::endl;
        std::cout <<"         -------------------------                                            /%%%%%%"<<std::endl;
        std::cout <<"         -------------------------                                          %%%%.  #%%%&"<<std::endl;
        std::cout <<"       || Que voulez vous faire ?  ||                                    %%%%%        &%%%"<<std::endl ;
        std::cout <<"       ||                          ||                                  &%%%%&#*,.  .*(%&%%%%*" <<std::endl;
        std::cout <<"       ||   1. Maps                ||                                &%%% /%            % *%%%*"<<std::endl;
        std::cout <<"       ||   2. Chemin speciaux     ||                              %%%%     &%        &(    /%%%"<<std::endl;
        std::cout <<"       ||   3. LiveCam             ||                            .%%%.        %.    %&        &%%%"<<std::endl;
        std::cout <<"       ||   4. Contact             ||                           %%%%           *%  %            %%%&"<<std::endl;
        std::cout <<"       ||   5. Quittez             ||                          *%%%   ./%&&&%%%%%%%%%%%%%%&&&%(,  .%%%"<<std::endl;
        std::cout <<"         -------------------------                            &%%%              #%  %               %%%("<<std::endl;
        std::cout <<"         -------------------------                           %%%  %/           %     *%           %& #%%&"<<std::endl;
        std::cout <<"                                                           .%%&    ,%        %#        &&        %     %%%"<<std::endl;
        std::cout <<"                                                          *%%&       &(    %&            %     &&       %%%"<<std::endl;
        std::cout <<"                                                         *%%&         .%  &               *%  %          %%%"<<std::endl;
        std::cout <<"                                                        ,%%%      (&&%%%%&%%%%&&&&&&&&&&%%%%%%%%&@&*      %%%"<<std::endl;
        std::cout <<"                                                        %%%&          %& .%               ,%  %          &#%%%"<<std::endl;
        std::cout <<"                                                       %%%  %        %     &%            %/    (%        % (%%%"<<std::endl;
        std::cout <<"                                                      #%%#  &&     %#        %         /%        &%     %.  &%%"<<std::endl;
        std::cout <<"                                                      %%%    &/  %&           &&      %,           %   &(   .%%&"<<std::endl;
        std::cout <<"                                                      %%%%%%&,%,%               %   %&              (%&/#&%%%%%%"<<std::endl;
        std::cout <<"                                                           %%%%%%%%%%%%%&&%(*,.  &&%  ..,*#&&&%%%%%%%%%%%&#"<<std::endl;
        std::cout <<"                                                                     .*%&&%%%%%%%%%%%%%%%%%%&&#,"<<std::endl;
        do
        {

            std::cin>>b;
            fflush(stdin);
            choix =b ;
        }while(choix !=49 && choix !=50 && choix !=51 && choix !=52 && choix !=53 );

        switch(choix)
        {
            case 49:
                system("cls");
                TrouverLeCheminLePlusCourt();
            break ;

            case 50:
                system("cls");
                TrouverLeCheminLePlusCourtSpecial(s);
            break ;

            case 51:
                system("cls");

                return 4 ;
            break ;

            case 52:
               system("cls");
               std::cout <<"                            Domaine skiable"<<std::endl;
               std::cout <<"                                LES ARCS    "<<std::endl;
               std::cout <<"                    Chalet des Villards - Arc 1800"<<std::endl;
               std::cout <<"                        73700 Les Arcs - France"<<std::endl;
               std::cout <<"                      Tel. +33 (0)4 79 04 24 00"<<std::endl;
               std::cout<<" "<< std::endl ;
               std::cout<<" "<< std::endl ;
               int p;
               char c;
               do
               {
                  std::cout <<" <----  retour menu ( ecrire 1)"<< std::endl;
                  std::cin>>c;
                  fflush(stdin);
                  p=c;
               }while(p!=49);

            break;

            case 53:
                exit(1);
            break;
        }

    }while(choix !=53 );
    return -1 ;
}


int main(int argc, char** argv)
{
    system("color F0");
    int a;
    do
    {
        a = menu();
    }while (a!=4);

    if (a==4)
    {
        cv::VideoCapture cap(0); //capture the video from web cam
        if (!cap.isOpened()) // if not success, exit program
        {
          return -1;
        }
        cv::Mat imgTmp;
        cap.read(imgTmp);
        cv::Mat imgLines = cv::Mat::zeros(imgTmp.size(), CV_8UC3);
        while (true)
        {
            cv::Mat imgRetourCam;
            bool bSuccess = cap.read(imgRetourCam); // read a new frame from video
            if (!bSuccess) //recommencer la vidéo
            {
                cap.set(cv::CAP_PROP_POS_FRAMES, 0);
                cap.read(imgRetourCam);
            }
            cv::namedWindow("Original", cv::WINDOW_NORMAL);
            cv::imshow("Original", imgRetourCam); //show the original image
            if (cv::waitKey(1) == 27)
                break;
        }
    }

    return 0;
}
