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

Skieur creationProfil()
{

    Skieur s;
    std::string _pseudo;
    std::string _mdp;
    std::cout << std::endl;
    std::cout << "---------- Bienvenue nous allons creer votre profil ----------" << std::endl;
    std::cout << std::endl;
    std::cout << "Veuillez entrer le pseudo de votre choix : " << std::endl;
    std::cout << std::endl;
    std::cin >> _pseudo;
    s.setPseudo(_pseudo);
    std::cout << std::endl;
    std::cout << "Veuillez a present creer un mot de passe : " << std::endl;
    std::cout << std::endl;
    std::cin >> _mdp;

    s.setMdp(_mdp);

    return s;
}

void archivageProfil(Skieur s) // Ecriture (sauvegarde) dans un fichier le pseudo et le mdp de l'utilisateur
{
    std::string const fichierProfil("compte.txt");
    std::ofstream monFlux(fichierProfil.c_str(), std::ios::app);
    monFlux << s.getPseudo() << std::endl;
    monFlux << s.getMdp() << std::endl;
    monFlux << s.getParam1()<< std::endl;
    monFlux << s.getParam2()<< std::endl;
}

std::string saisiePseudo()
{
    std::string _pseudo;
    std::cout << "                            ";
    std::cout << "Veuillez saisir votre pseudo : " << std::endl;
    std::cout << "                            ";
    std::cin >> _pseudo;
    std::cout<<std::endl;
    return _pseudo;
}

std::string saisieMdp()
{
    std::string _mdp;
    std::cout << "                            ";
    std::cout << "Veuillez saisir votre mot de passe : " <<std::endl;
    std::cout << "                            ";
    std::cin >> _mdp;

    return _mdp;
}

bool verifFichier(std::string _pseudo, std::string _mdp) // vérifier que le compte de l'utilisateur existe
{

    int i = 0;
    std::ifstream fichierProfil("compte.txt"); // ouverture du fichier en lectuer

    std::vector<std::string> Id;

    std::string ligne;

    while(getline(fichierProfil, ligne)) // remplissage du vector : ligne par ligne (une ligne dans une case)
    {
        Id.push_back(ligne);
    }

    while( (i <= Id.size()-1) )
    {

        if (_pseudo == Id[i] && _mdp == Id[i+1])
        {
            return true;
        }

        i+=2;
    }
    fichierProfil.close();
     return false;
}

bool Selection() // savoir si l'utilisateur veut se connecter ou créer un compte
{
    bool cond;

    int a = 0;
    char b;

    do
    {
        std::cout << "                            ";
        std::cout << "Bonjour, pour vous connecter entrez : 1 , " << std::endl;
        std::cout << "                            ";
        std::cout << "Pour creer un compte entrez : 2"<< std::endl;
       std::cout<<std::endl;
        std::cout << "                            ";
        std::cin >> b;
        std::cout<<std::endl;
        a=b;
         fflush(stdin);
    }while(a!=49 && a!=50);

    if(a == 49)
        cond = true;
    else
        cond = false;

    return cond;

}

Skieur connexionPage(Skieur& s) // on vérifie si le pseudo du joueur correspond a son mot de passe ou on lui créer un compte si il n'en possede pas (appel de la fonction créationProfil)
{
    int a = 1;

    if (Selection() == true)
    {


        do
        {
            s.setPseudo(saisiePseudo());
            s.setMdp(saisieMdp());
            std::string pseudo= s.getPseudo() ;
            std::string mdp = s.getMdp();
            if (verifFichier(pseudo ,mdp ) != true)
            {
                a=1;
                std::cout<<std::endl;
                std::cout << "                            ";
                std::cout << "Erreur d'authentification, veuillez reessayer ! " << std::endl;

            }
            else
            {
                 a = 0;
                return s;
            }

        }while(a != 0);
    }
    else
    {
        s = creationProfil();
        archivageProfil(s);
        return s;
    }
return s ;
}

void afficherSkieur(Skieur s)
{
    std::cout<<std::endl;
    std::cout << "                            ";
    std::cout << s.getPseudo() <<" Est connecte(e) " <<std::endl;
    std::cout << std::endl;
    Sleep(500);
     std::system("cls");
}

void sauvegardeParam(Skieur& s)
{
    std::cout << "HELLOOOOOOOO" << std::endl;

    std::string const nomFichier("compte.txt");
    std::ifstream monFlux1(nomFichier);
    std::string ligne;
    std::vector<std::string> Id;
    std::cout<< "lecture fichier en entier" <<std::endl;
    while(getline(monFlux1, ligne))
    {

        std::cout<< ligne <<std::endl;
        Id.push_back(ligne);
    }
    for(int i = 0; i < Id.size(); i++)
    {
        std::cout << Id[i] << std::endl;
    }
    monFlux1.close();

    std::cout<< "param 1 ; " << s.getParam1() << "  param 2 : " << s.getParam2()<< " pseudo : " << s.getPseudo() << std::endl;
    std::cout<<std::endl;

    for(int i = 0; i < Id.size(); i++)
    {


        if(Id[i] == s.getPseudo())
        {
            std::cout<< "im in the if" <<std::endl;

            Id[i+2] = s.getParam1();
            Id[i+3] = s.getParam2();
             std::cout<< "id2 : " << Id[i+2] << "  id 3 : " << Id[i+3]<<std::endl;
        }
    }
    for(int i = 0; i < Id.size(); i++)
    {
        std::cout << Id[i] << std::endl;
    }


    std::ofstream monFlux2(nomFichier.c_str());
    for(int i = 0; i < Id.size(); i++)
    {
        monFlux2 << Id[i] << std::endl;
    }




}

void choixParam(Skieur& s)
{
    std::string choix;
    bool cond = false;
    std::string newParam1;
    std::string newParam2;
    std::cout << "zeeeeeeeeeeeeeeeebiiiiiiiiiiii" <<std::endl;

    do{

        std::cout << " voulez vous modifier vos preferences ? 1 : oui / 2 : non " << std::endl;
        std::cin >> choix;
        if(choix == "1" || choix == "2")
        {
            cond = true;
        }
        else
            cond = false;


    }while(cond = false);

    std::string choix1;
    std::string choix2;
    bool cond1 = false;
    bool cond2 = false;

    if(choix == "1")
    {
        do {
            std::cout << "Entrez  ( 1 ) si vous souhaiter eviter les remontes et profiter au maximum des pistes et ( 0 ) si non : " << std::endl;
            std::cout <<std::endl;
            std::cout <<std::endl;
            std::cin >> choix1;
            std::cout <<std::endl;
            std::cout <<std::endl;
            if(choix1 == "1" || choix1 == "0")
                cond1 = true;

        }while(cond1 = false);

        if(choix1 == "1" )
        {
            newParam1 = "1";
            s.setParam1(newParam1);
        }
        else
        {
            newParam1 = "0";
            s.setParam1(newParam1);
        }

        do {
            std::cout << "Entrez  ( B ) pour eviter les pistes bleues "<< std::endl;
            std::cout << "Entrez  ( R ) pour eviter les pistes rouges "<< std::endl;
            std::cout << "Entrez  ( N ) pour eviter les pistes noires "<< std::endl;
            std::cout <<std::endl;
            std::cout <<std::endl;
            std::cin >> choix2;
            std::cout <<std::endl;
            std::cout <<std::endl;
            if(choix2 == "B" || choix2 == "R"|| choix2 == "N")
                cond2 = true;

        }while(cond2 = false);

        s.setParam2(choix2);

        std::cout<< "param 1 ; " << s.getParam1() << "  param 2 : " << s.getParam2()<<std::endl;
        sauvegardeParam(s);

        return;


    }
    else
    {
         return;
    }


}

/// -----------------------------------------------------------------------------------------------------------------------------
void print_route(std::vector<int> const &prev, int i, std::vector<Edge> crossedEdge)
{
    HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
    if (i < 0)
        {
        return;
        }


    print_route(prev, prev[i], crossedEdge);
    for(int x = 0; x < crossedEdge.size(); x++)
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

std::vector<Edge> EdgesTxt(std::vector<Node> nodes )
{
std::vector<Edge> edges;
Node start;
Node finish;
 int v1;//
    int v2;//
    int weight;//
     int num;///
    char edgeType;//
    std::string edgeName;//

    std::ifstream flxEdges("data_arcs.txt"); // ouverture du fichier arcs

    while(flxEdges)
    {
        flxEdges >> num >> edgeName >> edgeType >> v1 >> v2;
        //std::cout << num << "  " << edgeName << "  "  << edgeType << "  " << v1 << "  " << v2 <<std::endl;

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
                if(start.getName()=="arc1600"||start.getName()=="arc2000" && finish.getName()=="arc1600"|| finish.getName()=="arc2000")
                {
                    edge.setWeight(40);
                }
                  if(start.getName()=="arc1600"||start.getName()=="arc1800" && finish.getName()=="arc1600"|| finish.getName()=="arcarc1800")
                {
                    edge.setWeight(30);
                }


        edge.setNom(edgeName);
        edge.setNum(num);
        edges.push_back(edge);

    }
    for(int i = 0; i < edges.size(); i++)
    {
        edges[i].afficher();
    }
    std::cout<<std::endl;
return edges;
}

std::vector<Node> nodesTxt(int *v)
{
     std::vector<Node> nodes;

     int vertex;//
    int alt;//
    std::string nodeName;//
     std::ifstream flxPoints("data_points.txt"); // ouverture du fichier points

    flxPoints >> *v; // premiere ligne du texte  = nombre de sommet
    //std::cout << v << std::endl;
    while(flxPoints)  /// on creer la liste d'adjacence grace au valeur r�cup�r�es dans le fichier
    {
            flxPoints >> vertex >> nodeName >> alt;
            Node sommet(vertex, nodeName, alt);
            nodes.push_back(sommet);
            //std::cout << vertex << "  " << nodeName << "  "  << alt << "  " << std::endl;
    }
return nodes ;
}


void AllShortestPast(Graph const &graph, Node source, int v,std::vector<Edge> edges) /// Dijkstra tous les plus courts chemins
{
    // prendre la source comme arrete = 0
    std::priority_queue<Node, std::vector<Node>, comp> min_heap;
    min_heap.push({source.getVertex(), source.getName(), 0});


    std::vector<Node> nodes;
    int z=0;
    nodes = nodesTxt(&z);
    edges = EdgesTxt(nodes);
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
            std::cout << "  Heure d'arrive estimee a : " <<heure << "h"<<  minEstime;
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


void findShortestPaths(Graph const &graph, Node source, int v, Node fin, std::vector<Edge> edges) /// Dijkstra plus court chemins choix du depart + arrivee
{
    // prendre la source comme arrete = 0
    std::priority_queue<Node, std::vector<Node>, comp> min_heap;
    min_heap.push({source.getVertex(), source.getName(), 0});
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


        for (auto i: graph.m_adjList[u])     /// <------------
        {
            int v = i.getDest().getVertex();
            int weight = i.getWeight();
            //int w = i.get


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
            std::cout << "  Heure d'arrive estimee a : " <<heure << "h"<<  minEstime;
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


void CheminBFS(std::vector< std::vector<Edge> > adjList, Node start,Node finish)
{
std::cout << "Chemin : ";

    int n = adjList.size();
    // Create a "visited" array (true or false) to keep track of if we visited a vertex.
        bool visited[n] = { false };
        std::vector<int>ListeSommet;

    // Create a queue for the nodes we visit.
    std::queue<Node> q;

    // Add the starting vertex to the queue and mark it as visited.
    q.push(start);
    visited[start.getVertex()] = true;
int fin = 0;
    // While the queue is not empty..
    while(q.empty() == false && fin!=1)
        {
        Node ver = q.front();
        int vertex=ver.getVertex();
        ListeSommet.push_back(vertex);

            q.pop();

        // Doing +1 in the cout because our graph is 1-based indexing, but our code is 0-based.

        // Loop through all of it's friends.
        for(int i = 0; i < adjList[vertex].size(); i++)
            {
            // If the friend hasn't been visited yet, add it to the queue and mark it as visited
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
                                       //EKIP
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
    }// Given an Adjacency List, do a BFS on vertex "start"

void AdjListBFS(std::vector< std::vector<Edge> > adjList, Node start)
    {
    std::cout << "Doing a BFS on an adjacency list : ";

    int n = adjList.size();
    // Create a "visited" array (true or false) to keep track of if we visited a vertex.
    bool visited[n] = { false };

    // Create a queue for the nodes we visit.
    std::queue<Node> q;

    // Add the starting vertex to the queue and mark it as visited.
    q.push(start);
    visited[start.getVertex()] = true;

    // While the queue is not empty..
    while(q.empty() == false)
        {
        Node ver = q.front();
         int vertex=ver.getVertex();
        q.pop();

        // Doing +1 in the cout because our graph is 1-based indexing, but our code is 0-based.
        std::cout << vertex << " ";

        // Loop through all of it's friends.
        for(int i = 0; i < adjList[vertex].size(); i++)
            {
            // If the friend hasn't been visited yet, add it to the queue and mark it as visited
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
    //edges.push_back({v1,v2,poids});
    // construct graph

   int choix ;
    char b ;
    std::cout<<""<<std::endl;
    do
    {
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
       do {
           std::cout <<" De quel sommet voulez vous partir ?"<<std::endl;
           std::cin >>saisieSource;

           }while(saisieSource<1);
   }while(saisieSource>37);
   do{
    do {
          std::cout <<" Quel est le sommet d'arrive de votre choix ?"<<std::endl;
    std::cin>> saisieFin;
    }while(saisieFin<1);
   }while(saisieFin>37);

    source = nodes[saisieSource-1];
    fin = nodes[saisieFin-1];

    findShortestPaths(graph, source, v , fin, edges);

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

        AllShortestPast(graph, source, v, edges);

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
       do {
           std::cout <<" De quel sommet voulez vous partir ?"<<std::endl;
           std::cin >>saisieSource;

           }while(saisieSource<1);
   }while(saisieSource>37);
   do{
    do {
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
int menu ()
{
    int choix ;
    char b ;
     Skieur s;
    Skieur s;
    afficherSkieur(connexionPage(s));
    choixParam(s);
    std::cout<<""<<std::endl;
    do
    {
        std::cout<<""<<std::endl;
std::cout<<""<<std::endl;
std::cout<<""<<std::endl;
std::cout<<""<<std::endl;
std::cout<<""<<std::endl;
    std::cout <<"         -------------------------                                            /%%%%%%"<<std::endl;
    std::cout <<"         -------------------------                                          %%%%.  #%%%&"<<std::endl;
    std::cout <<"       || Que voulez vous faire ?  ||                                    %%%%%        &%%%"<<std::endl ;
    std::cout <<"       ||                          ||                                  &%%%%&#*,.  .*(%&%%%%*" <<std::endl;
    std::cout <<"       ||   1. Maps                ||                                &%%% /%            % *%%%*"<<std::endl;
    std::cout <<"       ||   2. Chemin speciaux     ||                              %%%%     &%        &(    /%%%"<<std::endl;
    std::cout <<"       ||   3. ---                 ||                            .%%%.        %.    %&        &%%%"<<std::endl;
    std::cout <<"       ||   4.Arret                ||                           %%%%           *%  %            %%%&"<<std::endl;
    std::cout <<"         -------------------------                            *%%%   ./%&&&%%%%%%%%%%%%%%&&&%(,  .%%%"<<std::endl;
    std::cout <<"         -------------------------                           &%%%              #%  %               %%%("<<std::endl;
    std::cout <<"                                                             %%%  %/           %     *%           %& #%%&"<<std::endl;
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
   }while(choix !=49 && choix !=50 && choix !=51 && choix !=52 );

   switch(choix)
   {
   case 49:
       system("cls");
std::cout <<" chemin  le plus court " << std::endl;
TrouverLeCheminLePlusCourt();
   break ;

   case 50:
       system("cls");
   std:: cout<<"Quel deck voulez vous modifier " << std::endl ;


   break ;

   case 51:
       system("cls");
   std:: cout<<" cam " << std::endl ;
return 4 ;
   break ;

   case 52:
   exit(1);
   break;


   }

  }while(choix !=52 );
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
