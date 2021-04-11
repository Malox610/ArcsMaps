#ifndef SKIEUR_H_INCLUDED
#define SKIEUR_H_INCLUDED
#include "string"

class Skieur
{
private :

    std::string m_pseudo;
    std::string m_mdp;
    std::string m_param1 = "0";
    char m_param2 = 79;

public :

    Skieur();
    ~Skieur();
    std::string getPseudo();
    std::string getMdp();
    std::string getParam1();
    char getParam2();
    void setPseudo(std::string _pseudo);
    void setMdp(std::string _mdp);
    void setParam1(std::string newParam1);
    void setParam2(std::string newParam2);
    void afficher();
    void afficherSkieur();
    void sauvegardeParam();
    void choixParam();
    void archivageProfil();
    void creationProfil();
    bool verifFichier(std::string _pseudo, std::string _mdp); // vérifier que le compte de l'utilisateur existe
    void connexionPage();


};

#endif // SKIEUR_H_INCLUDED
