#include <iostream>
#include <windows.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

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

      cin>>b;
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
    return 0;
}
