/*
    Example for using TwoBarLinakge for IK, FK and PathPlanning
    Author: Kashish Goyal
*/

#include <iostream>
#include "TwoBarLinkage.hpp"

int main()
{
    double l1 = 1, l2 = 1;
    std::cout << "Create a new chain with Link1 length " << l1 << " units and Link2 length " << l2 << " units." << std::endl;
    TwoBarLinkage chain(l1, l2, 100);
    std::cout << "*****Forward Kinematics********" << std::endl;
    E_Point fk(chain.forwardKin(Theta_List{1.57, 0}));
    std::cout << fk[0] << ", " << fk[1] << std::endl;

    std::cout << "*****Inverse Kinematics********" << std::endl;
    std::vector<Theta_List> ik(chain.inverseKin(E_Point{0.5, 0.5}));
    if (ik.size())
    {
        std::cout << ik[0][0] << ", " << ik[0][1] << std::endl;
        std::cout << ik[1][0] << ", " << ik[1][1] << std::endl;
    }
    else
    {
        std::cout << "Unable to find feasible IK solution" << std::endl;
    }

    std::cout << "*****Path Planning in C SPACE********" << std::endl;
    std::vector<Theta_List> path1 = chain.get_path_c_space(E_Point{1, 1}, E_Point{1, -1});
    if (!path1.size())
    {
        std::cout << "Unable to find feasible path" << std::endl;
    }
    for (auto i : path1)
    {
        std::cout << ' ' << i[0] << ',' << i[1] << std::endl;
    }

    std::cout << "*****Path Planning in E SPACE********" << std::endl;
    std::vector<Theta_List> path2 = chain.get_path_e_space(E_Point{1, 1}, E_Point{1, -1});
    if (!path2.size())
    {
        std::cout << "Unable to find feasible path" << std::endl;
    }
    for (auto i : path2)
    {
        std::cout << ' ' << i[0] << ',' << i[1] << std::endl;
    }

    std::cout << "*****Path Planning in C SPACE with Obstacle********" << std::endl;
    chain.add_line_obstacle({0, 0}, {2, 0});
    path1 = chain.get_path_c_space(E_Point{1, 1}, E_Point{1, -1});
    if (!path1.size())
    {
        std::cout << "Unable to find feasible path" << std::endl;
    }
    for (auto i : path1)
    {
        std::cout << ' ' << i[0] << ',' << i[1] << std::endl;
    }

    std::cout << "*****Path Planning in E SPACE with Obstacle********" << std::endl;
    path2 = chain.get_path_e_space(E_Point{1, 1}, E_Point{1, -1});
    if (!path2.size())
    {
        std::cout << "Unable to find feasible path" << std::endl;
    }
    for (auto i : path2)
    {
        std::cout << ' ' << i[0] << ',' << i[1] << std::endl;
    }

    return 0;
}