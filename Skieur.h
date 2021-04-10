#ifndef SKIEUR_H_INCLUDED
#define SKIEUR_H_INCLUDED
#include "string"

class Skieur
{
private :

    std::string m_pseudo;
    std::string m_mdp;

public :


    std::string getPseudo();
    std::string getMdp();
    void setPseudo(std::string _pseudo);
    void setMdp(std::string _mdp);
    void afficher();
};

#endif // SKIEUR_H_INCLUDED
