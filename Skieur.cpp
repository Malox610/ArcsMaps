#include "Skieur.h"
#include <iostream>
#include <string>

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
