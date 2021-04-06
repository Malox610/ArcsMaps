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
   std:: cout<<"partie lance " << std::endl ; /// A modifier quand on aura le programme

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
 }
 /// suite a mettre en sous programme qui sera appelé dans menu
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
