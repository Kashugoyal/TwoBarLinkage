/*
    This file contains the TwoBarLinkage class declaration.
    The two barlinkage provides both FK and IK functionality along with Path Planning
    Author: Kashish Goyal
*/

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
        The path planning is done in the configuration space i.e theta1 in [0, 2pi] and theta2 in [0, 2Pi]
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
        The path planning is done in the euclidian space i.e X in [-l1 -l2, l1 + l2] and Y in [-l1 -l2, l1 + l2]
        The function uses A* algorithm for finding the solution.
        @params: 
            E_point: euclidian point/ starting point for the end effector 
            E_point: euclidian point/ target point for the end effector 
        @return:
            std::vector<Theta_List>: an array of joint angles for the linkage to follow to achieve target position
    */
    std::vector<Theta_List> get_path_e_space(const E_Point &source, const E_Point &target);

    /*
        Function to add a line obstacle to the linkage workspace. The path planner will try to generate a path that avoids the obstacle
        Existing obstacle is replaced.
        @params: 
            E_point: euclidian point/ start for the line obstacle
            E_point: euclidian point/ end for the line obstacle
        @return: None
    */
    void add_line_obstacle(E_Point start, E_Point end);

    /*
        Function to remove existing obstacle from the linkage workspace
        @params: None
        @return: None
    */
    void remove_obstacle();

protected:
    /*
        Utility Function to find out if the Euclidian point is within the robot's reach
        @params: 
            E_point: euclidian point/ target for the end effector 
        @return:
            bool: true or false as result
    */
    inline bool isReachable(const E_Point &);

    /*
        Utility Function to convert angles from [-Pi , Pi] to the range [0, 2 Pi]
        @params: 
            double: angle in radians
        @return:
            double: angle in radians
    */
    inline double absoluteAngle(double angle);

    /*
        Internal Function to initialize the planner
        @params: None 
        @return: None
    */
    void initialize_planner();

    /*
        Internal Function to convert the line obstacle to a configuration space obstacle
        @params: None 
        @return: None
    */
    void add_c_space_obstacle();

    /*
        Internal Function to convert the line obstacle to a euclidian space obstacle
        @params: None 
        @return: None
    */
    void add_e_space_obstacle();

    PathPlanner::Generator planner;
    std::array<E_Point, 2> obstacle;
    int resolution;

private:
    double l1;
    double l2;
};

#endif // TwoBarLinkage_HPP