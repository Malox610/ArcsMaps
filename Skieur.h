#ifndef SKIEUR_H_INCLUDED
#define SKIEUR_H_INCLUDED
#include "string"

class Skieur
{
private :

    std::string m_pseudo;
    std::string m_mdp;
    std::string m_param1 = "0";
    std::string m_param2 = "O";

public :


    std::string getPseudo();
    std::string getMdp();
    std::string getParam1();
    std::string getParam2();
    void setPseudo(std::string _pseudo);
    void setMdp(std::string _mdp);
    void setParam1(std::string newParam1);
    void setParam2(std::string newParam2);
    void afficher();

};

#endif // SKIEUR_H_INCLUDED
