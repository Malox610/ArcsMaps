#include "Skieur.h"
#include <iostream>
#include <string>
#include <windows.h>
#include <fstream>
#include <sstream>
#include <vector>

Skieur::Skieur()
{
    connexionPage();
    choixParam();
}
Skieur::~Skieur()
{

}
std::string Skieur::getPseudo()
{
    return m_pseudo;
}

std::string Skieur::getMdp()
{
    return m_mdp;
}

std::string Skieur::getParam1()
{
    return m_param1;
}

char Skieur::getParam2()
{
    return m_param2;
}

void Skieur::setPseudo(std::string _pseudo)
{
    m_pseudo = _pseudo;
}

void Skieur::setMdp(std::string _mdp)
{
    m_mdp = _mdp;
}

void Skieur::setParam1(std::string newParam1)
{
    m_param1 = newParam1;
}

void Skieur::setParam2(char newParam2)
{
    m_param2 = newParam2;

}


void Skieur::afficher()
{
    std::cout << "          -----------  Profil de : " << getPseudo() << " -----------"<< std::endl;
    std::cout << std::endl;
    std::cout << std::endl;

}

void Skieur::archivageProfil()
{
    std::string const fichierProfil("compte.txt");
    std::ofstream monFlux(fichierProfil.c_str(), std::ios::app);
    monFlux << getPseudo() << std::endl;
    monFlux << getMdp() << std::endl;
    monFlux << getParam1()<< std::endl;
    monFlux << getParam2()<< std::endl;
}

void Skieur::afficherSkieur()
{
    std::cout<<std::endl;
    std::cout << "                            ";
    std::cout << getPseudo() <<" Est connecte(e) " <<std::endl;
    std::cout << std::endl;
    Sleep(1000);
     std::system("cls");
}

void Skieur::sauvegardeParam()
{
    std::string const nomFichier("compte.txt");
    std::ifstream monFlux1(nomFichier);
    std::string ligne;
    std::vector<std::string> Id;
    while(getline(monFlux1, ligne))
    {
        Id.push_back(ligne);
    }

    monFlux1.close();

    std::cout<< "vos parametre actuel sont "<< getParam2()<<std::endl;
    int taille=Id.size();
    for(int i = 0; i < taille; i++)
    { ///plante dans cette boucle si c'est pas jps comme pseudo mais sauvegarde quand meme donc au lancement suivant on a la piste choisi quand ca a crash
        if(Id[i] == getPseudo())
        {
            Id[i+2] = getParam1();
            Id[i+3] = getParam2();

        }
    }

    std::ofstream monFlux2(nomFichier.c_str());

    for(int i = 0; i < taille; i++)
    {
         std::cout <<"bonjour 3 " ;
        monFlux2 << Id[i] << std::endl;
    }
}

void Skieur::choixParam()
{
    std::string choix;
    char cond = false;
    std::string newParam1;

    do{
        //std::cout<<" Votre preference actuel est : " << getParam2()<<std::endl;
        std::cout << " voulez vous modifier vos preferences ? 1 : oui / 2 : non " << std::endl;
        std::cin >> choix;
        if(choix == "1" || choix == "2")
        {
            cond = true;
        }
        else
            cond = false;


    }while(cond == false);

    std::string choix1;
    char choix2;
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

        }while(cond1 == false);

        if(choix1 == "1" )
        {
            newParam1 = "1";
            setParam1(newParam1);
            cond2=true;
        }
        else
        {
            newParam1 = "0";
            setParam1(newParam1);
        }
     if (cond2==(true))
     {
      char b;
     int secu;
      do
      {
        std::cout<<std::endl;
        std::cout<<std::endl;
        std::cout<<std::endl;
        std::cout<<std::endl;
        std::cout<<std::endl;
        std::cout <<"         ------------------------------                                     /%%%%%%"<<std::endl;
        std::cout <<"         ------------------------------                                   %%%%.  #%%%&"<<std::endl;
        std::cout <<"       || Que voulez vous faire ?      ||                               %%%%%        &%%%"<<std::endl ;
        std::cout <<"       ||                              ||                             &%%%%&#*,.  .*(%&%%%%*" <<std::endl;
        std::cout <<"       ||   1.Eviter les pistes bleues ||                           &%%% /%            % *%%%*"<<std::endl;
        std::cout <<"       ||   2.Eviter les pistes rouges ||                          %%%%     &%        &(    /%%%"<<std::endl;
        std::cout <<"       ||   3.Eviter les pistes noires ||                        .%%%.        %.    %&        &%%%"<<std::endl;
        std::cout <<"       ||   4. Ne rien eviter          ||                       %%%%           *%  %            %%%&"<<std::endl;
        std::cout <<"         ------------------------------                       *%%%   ./%&&&%%%%%%%%%%%%%%&&&%(,  .%%%"<<std::endl;
        std::cout <<"         ------------------------------                      &%%%              #%  %               %%%("<<std::endl;
        std::cout <<"                                                            %%%  %/           %     *%           %& #%%&"<<std::endl;
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
        std::cout <<std::endl;
        std::cout <<std::endl;
        std::cout<<std::endl;
        std::cout<<std::endl;
        std::cin>>b;
        fflush(stdin);
        secu =b ;
      }while(secu!=49 && secu!=50 && secu !=51 && secu !=52 );
     switch(secu)
     {
       case 49:
           choix2=66;
        break;
        case 50:
           choix2=82;
        break;
        case 51:
           choix2=78;
        break;
        case 52:
           choix2=79;
        break;


     }
    setParam2(choix2);
     }

        sauvegardeParam();
        std::cout<<" bonjour4 ";
    }
}
bool Skieur::verifFichier(std::string _pseudo, std::string _mdp) // vérifier que le compte de l'utilisateur existe
{

    int i = 0;
    std::ifstream fichierProfil("compte.txt"); // ouverture du fichier en lectuer

    std::vector<std::string> Id;

    std::string ligne;


    while(getline(fichierProfil, ligne)) // remplissage du vector : ligne par ligne (une ligne dans une case)
    {
        Id.push_back(ligne);
    }
    int taille =Id.size();
    while( (i <= taille-1) )
    {

        if (_pseudo == Id[i] && _mdp == Id[i+1])
        {
            std::istringstream iss( Id[i+3] ); ///permet de recup parametre tant qu'on y est
            int _param2;
            iss>> _param2; ///convertie un string en int oiur ensuite mettre en char pour param2
            setParam2(_param2);

            return true;
        }

        i+=2;
    }
    fichierProfil.close();
     return false;
}

void Skieur::creationProfil()
{


    std::string _pseudo;
    std::string _mdp;
    std::cout << std::endl;
    std::cout << "---------- Bienvenue nous allons creer votre profil ----------" << std::endl;
    std::cout << std::endl;
    std::cout << "Veuillez entrer le pseudo de votre choix : " << std::endl;
    std::cout << std::endl;
    std::cin >> _pseudo;
    setPseudo(_pseudo);
    std::cout << std::endl;
    std::cout << "Veuillez a present creer un mot de passe : " << std::endl;
    std::cout << std::endl;
    std::cin >> _mdp;
    setMdp(_mdp);

}
void Skieur::connexionPage()
{
    bool cond;
    int a=1;
    int c = 0;
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
        c=b;
         fflush(stdin);
    }while(c!=49 && c!=50);

    if(c == 49)
        cond = true;
    else
        cond = false;

    if (cond == true)
    {


        do
        {
            std::string _pseudo;
            std::cout << "                            ";
            std::cout << "Veuillez saisir votre pseudo : " << std::endl;
            std::cout << "                            ";
            std::cin >> _pseudo;
            std::cout<<std::endl;
            setPseudo(_pseudo);

            std::string _mdp;
            std::cout << "                            ";
            std::cout << "Veuillez saisir votre mot de passe : " <<std::endl;
            std::cout << "                            ";
            std::cin >> _mdp;
            setMdp(_mdp);
            if (verifFichier(_pseudo ,_mdp ) != true)
            {
                a=1;
                std::cout<<std::endl;
                std::cout << "                            ";
                std::cout << "Erreur d'authentification, veuillez reessayer ! " << std::endl;

            }
            else
            {
                 a = 0;
                  std::cout << "Connexion reussi ! " << std::endl;

            }

        }while(a != 0);
    }
    else
    {
        creationProfil();
        archivageProfil();
    }


}

