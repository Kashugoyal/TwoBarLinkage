#ifndef TwoBarLinkage_HPP
#define TwoBarLinkage_HPP

#include <array>
#include <vector>
#include "path_planner.hpp"

#include "Line.hpp"
typedef std::array<double, 2> Theta_List;

/*
    TwoBarLinkage
    class to implement the functionality for a two bar linkage with 2 dof
*/
class TwoBarLinkage
{
public:
    /*
        TwoBarLinkage Constructor
        @params: 
            double: length of first link, 
            double: length of second link,
            int: resolution for path planning
    */
    TwoBarLinkage(double, double, int);

    /*
        TwoBarLinkage Constructor, sets defulats - unit length links and resolution of 100
        @params: None
    */
    TwoBarLinkage();

    virtual ~TwoBarLinkage();

    /*
        Function to solve the forward kinematics for the two bar linkage
        @params: 
            Theta_List: an array of joint angles for the linkage
        @return:
            E_point: euclidian point/ target for the end effector 
    */
    E_Point forwardKin(const Theta_List &);

    /*
        Function to solve the inverse kinematics for the two bar linkage
        @params: 
            E_point: euclidian point/ target for the end effector 
        @return:
            std::vector<Theta_List>: list of possible IK solutions for the given end effector position
    */
    std::vector<Theta_List> inverseKin(const E_Point &);

    /*
        Function to solve the path planning problem for the two bar linkage
        Thgej path planning is done inthe configuration space i.e [0, 2pi], [0, 2Pi]
        The function uses A* algorithm for finding the solution.
        @params: 
            E_point: euclidian point/ starting point for the end effector 
            E_point: euclidian point/ target point for the end effector 
        @return:
            std::vector<Theta_List>: an array of joint angles for the linkage to follow to achieve target position
    */
    std::vector<Theta_List> get_path_c_space(const E_Point &source, const E_Point &target);

    /*
        Function to solve the path planning problem for the two bar linkage
        Thgej path planning is done inthe configuration space i.e [0, 2pi], [0, 2Pi]
        The function uses A* algorithm for finding the solution.
        @params: 
            E_point: euclidian point/ starting point for the end effector 
            E_point: euclidian point/ target point for the end effector 
        @return:
            std::vector<Theta_List>: an array of joint angles for the linkage to follow to achieve target position
    */
    std::vector<Theta_List> get_path_e_space(const E_Point &source, const E_Point &target);

    /*
        Function to solve the forward kinematics for the two bar linkage
        @params: 
            Theta_List: an array of joint angles for the linkage
        @return:
            E_point: euclidian point/ target for the end effector 
    */
    void add_line_obstacle(E_Point start, E_Point end);

    /*
        Function to solve the forward kinematics for the two bar linkage
        @params: 
            Theta_List: an array of joint angles for the linkage
        @return:
            E_point: euclidian point/ target for the end effector 
    */
    void remove_obstacle();

protected:
    inline bool isReachable(const E_Point &);
    inline double absoluteAngle(double angle);
    void initialize_planner();
    void add_c_space_obstacle();
    void add_e_space_obstacle();
    PathPlanner::Generator planner;

private:
    double l1;
    double l2;
    int resolution;
    std::array<E_Point, 2> obstacle;
};

#endif // TwoBarLinkage_HPP