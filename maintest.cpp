/*
    source Dijkstra : https://www.techiedelight.com/single-source-shortest-paths-dijkstras-algorithm/
*/
#include <iostream>
#include <vector>
#include <queue>
#include <climits>
#include <fstream>
#include <windows.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include "Graphe.h"
#include "Edge.h"
#include "Node.h"

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
                   message="la piste noir ";
                    break ;

                    case 82 : //piste rouge
                       SetConsoleTextAttribute(hConsole, 244);
                   message="la piste rouge ";
                    break ;

                    case 66 : //piste bleu
                 SetConsoleTextAttribute(hConsole, 241);
                   message="la piste bleu ";
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

        weight = abs(start.getWeight()- finish.getWeight());
        Edge edge(start, finish, weight);
        edge.setNom(edgeName);
        edge.setType(edgeType);
        edge.setNum(num);
        edges.push_back(edge);

    }
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


void AllShortestPast(Graph const &graph, Node source, int v,s td::vector<Edge> edges) /// Dijkstra tous les plus courts chemins
{
    // prendre la source comme arrete = 0
    std::priority_queue<Node, std::vector<Node>, comp> min_heap;
    min_heap.push({source.getVertex(), source.getName(), 0});

    std::vector<Edge> edges;
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
            std::cout << "Chemin (" << source.getName() << " --> " << i << "): Poids minimal = "
                 << dist[i] << ", Route = [ ";
            print_route(prev, i, crossedEdge);
            std::cout << "]" << std::endl;
    }
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
                std::cout << u << std::endl;
            }


        }


        // marquage des sommets
        done[u] = true;
    }

    for (int i = 0; i < v; i++)
    {
        if (i != source.getVertex() && dist[i] != INT_MAX && i == fin.getVertex())
        {
            std::cout << "Chemin (" << source.getName() << " --> " << i << "): Poids minimal = "
                 << dist[i] << ", Route = [ ";
            print_route(prev, i, crossedEdge);
            std::cout << "]" << std::endl;

        }

    }
}

// Given an Adjacency List, do a BFS on vertex "start"

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
    std::cout << std::endl ;
    return;
}


void TrouverLeCheminLePlusCourt()
   {


     int v = 0 ;//
    std::string nodeName;//
    std::string edgeName;
    std::vector<Node> nodes;
    std::vector<Edge> edges;
      std::cout<<"bonjour";



    //edges.push_back({v1,v2,poids});
    // construct graph


    int saisieSource;
    int saisieFin;
    Node source ;
    Node fin ;

   do{
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
   nodes = nodesTxt(&v);
    edges = EdgesTxt(nodes);
     Graph graph(edges, 95);


    source = nodes[saisieSource-1];

    fin = nodes[saisieFin-1];


    findShortestPaths(graph, source, v , fin);

//AllShortestPast(graph, source, v);
    //AdjListBFS(graph.m_adjList, source);

    /*for(int i = 0; i < nodes.size()-1; i++)
    {
        nodes[i].afficher();
    }

    for(int i = 0; i < edges.size()-1; i++)
    {
        edges[i].afficher();
    }*/

   }
int menu ()
{
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
