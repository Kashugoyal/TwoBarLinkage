#include <iostream>
#include "TwoBarLinkage.hpp"

int main()
{
    TwoBarLinkage chain;
    // TwoBarLinkage chain;
    E_Point fk(chain.forwardKin(Theta_List {1.57, 0}));
    std::vector<Theta_List> ik(chain.inverseKin(E_Point {3, 2}));

    std::cout << "*****Path Planning in C SPACE********" << std::endl;

    std::vector<Theta_List> path1 = chain.get_path_c_space(E_Point {1, 1}, E_Point {1, -1});
    for (auto i : path1) {
        std::cout << ' ' << i[0] << ',' << i[1] << std::endl;
    }
    std::cout << "*************" << std::endl;
    std::cout << "*****Path Planning in E SPACE********" << std::endl;
    std::vector<Theta_List> path2 = chain.get_path_e_space(E_Point {1, 1}, E_Point {1, -1});
    for (auto i : path2) {
        std::cout << ' ' << i[0] << ',' << i[1] << std::endl;
    }
    std::cout << "*************" << std::endl;
    std::cout << "*****Path Planning in C SPACE with Obstacle********" << std::endl;
    
    chain.add_line_obstacle({0,0}, {2, 0});
    path1 = chain.get_path_c_space(E_Point {1, 1}, E_Point {1, -1});
    for (auto i : path1) {
        std::cout << ' ' << i[0] << ',' << i[1] << std::endl;
    }

    std::cout << "*************" << std::endl;
    std::cout << "*****Path Planning in C SPACE with Obstacle********" << std::endl;

    path2 = chain.get_path_e_space(E_Point {1, 1}, E_Point {1, -1});
    for (auto i : path2) {
        std::cout << ' ' << i[0] << ',' << i[1] << std::endl;
    }
    std::cout << "*************" << std::endl;

    return 0;
}