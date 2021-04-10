#ifndef SKIEUR_H_INCLUDED
#define SKIEUR_H_INCLUDED
#include "string"

class Skieur
{
private :

    std::string m_pseudo;
    std::string m_mdp;
    bool m_param1 = 0;
    char m_param2;

public :


    std::string getPseudo();
    std::string getMdp();
    bool getParam1();
    char getParam2();
    void setPseudo(std::string _pseudo);
    void setMdp(std::string _mdp);
    void setParam1(bool newParam1);
    void setParam2(char newParam2);
    void afficher();
};

#endif // SKIEUR_H_INCLUDED
