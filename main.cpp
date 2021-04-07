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
using namespace cv;

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


void findShortestPaths(Graph const &graph, Node source, int v, Node fin)
{
    // prendre la source comme arrete = 0
    std::priority_queue<Node, std::vector<Node>, comp> min_heap;
    min_heap.push({source.getVertex(), source.getName(), 0});

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
            print_route(prev, i);
            std::cout << "]" << std::endl;

        }
    }
}



void TrouverLeCheminLePlusCourt()
{
    int v;
    int v1;
    int v2;
    Node start;
    Node finish;
    int weight;
    int vertex;
    int alt;
    int num;
    std::string edgeType;
    std::string nodeName;
    std::string edgeName;
    std::vector<Node> nodes;
    std::vector<Edge> edges;

    std::ifstream flxPoints("data_points.txt"); // ouverture du fichier points

    flxPoints >> v; // premiere ligne du texte  = nombre de sommet
    //std::cout << v << std::endl;
    while(flxPoints)  /// on creer la liste d'adjacence grace au valeur r�cup�r�es dans le fichier
    {
            flxPoints >> vertex >> nodeName >> alt;
            Node sommet(vertex, nodeName, alt);
            nodes.push_back(sommet);
            //std::cout << vertex << "  " << nodeName << "  "  << alt << "  " << std::endl;
    }

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



    //edges.push_back({v1,v2,poids});




    // construct graph
    Graph graph(edges, 95);
    int saisieSource;
    int saisieFin;
    Node source ;
    Node fin ;
    std::cout <<" De quel sommet voulez vous partir ?"<<std::endl;
    std::cin >>saisieSource;
    std::cout <<" Quel est le sommet d'arrive de votre choix ?"<<std::endl;
    std::cin>> saisieFin;
    source = nodes[saisieSource-1];
    fin = nodes[saisieFin-1];
    findShortestPaths(graph, source, v , fin);

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
   do
   {

      std::cin>>b;
      fflush(stdin);
      choix =b ;
   }while(choix !=49 && choix !=50 && choix !=51 && choix !=52 );

   switch(choix)
   {
   case 49:
  
    TrouverLeCheminLePlusCourt();
   break ;

   case 50:
   std:: cout<<"Quel deck voulez vous modifier " << std::endl ;


   break ;

   case 51:
   std:: cout<<" cam " << std::endl ; /// A modifier quand on aura le programme
return 4 ;
   break ;

   case 52:
   exit(1);
   break;


   }

  }while(choix !=52 );
}
int main(int argc, char** argv)
{
 system("color F0");
     int a;
    do
    {
       a = menu();
    }while (a!=4);

    VideoCapture cap(0); //capture the video from web cam
 if (!cap.isOpened()) // if not success, exit program
 {
 return -1;
 }
  Mat imgTmp;
cap.read(imgTmp);
Mat imgLines = Mat::zeros(imgTmp.size(), CV_8UC3);
 while (true)
 {
 Mat imgRetourCam;
 bool bSuccess = cap.read(imgRetourCam); // read a new frame from video
 if (!bSuccess) //recommencer la vidéo
 {
 cap.set(CAP_PROP_POS_FRAMES, 0);
cap.read(imgRetourCam);
 }
 namedWindow("Original", WINDOW_NORMAL);
 imshow("Original", imgRetourCam); //show the original image
if (waitKey(1) == 27)
 break;
    
    return 0;
}
