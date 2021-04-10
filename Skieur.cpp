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

void Skieur::setPseudo(std::string _pseudo)
{
    m_pseudo = _pseudo;
}

void Skieur::setMdp(std::string _mdp)
{
    m_mdp = _mdp;
}

void Skieur::afficher()
{
    std::cout << "          -----------  Profil de : " << getPseudo() << " -----------"<< std::endl;
    std::cout << std::endl;
    std::cout << std::endl;

}
