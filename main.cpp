#include <iostream>
#include "TwoBarLinkage.hpp"

int main()
{
    // TwoBarLinkage chain(3, 5, 100);
    TwoBarLinkage chain;
    E_Point fk(chain.forwardKin(Theta_List {1.57, 0}));
    std::vector<Theta_List> ik(chain.inverseKin(E_Point {3, 2}));

    for (auto i : fk) {
        std::cout << ' ' << i;
    }
    std::cout << std::endl;
    for (auto i : ik) {
        std::cout << ' ' << i[0] << ',' << i[1] << std::endl;
    }
    std::vector<Theta_List> path1 = chain.get_path_c_space(E_Point {1, 1}, E_Point {1, -1});
    
    std::cout << "*************" << std::endl;
    for (auto i : path1) {
        std::cout << ' ' << i[0] << ',' << i[1] << std::endl;
    }

    chain.add_line_obstacle({0,0}, {2, 0});
    std::vector<Theta_List> path2 = chain.get_path_e_space(E_Point {1, 1}, E_Point {1, -1});
    std::cout << "*************" << std::endl;
    for (auto i : path2) {
        std::cout << ' ' << i[0] << ',' << i[1] << std::endl;
    }

    return 0;
}