/*
    This file implements TwoBarLinkage.
*/

#include <cmath>
#include "TwoBarLinkage.hpp"
#define sq(a) pow(a, 2)

TwoBarLinkage::TwoBarLinkage(double l1, double l2, int resolution) : l1(l1), l2(l2), resolution(resolution)
{
}

TwoBarLinkage::TwoBarLinkage() : l1(1), l2(1), resolution(100)
{
}

void TwoBarLinkage::initialize_planner()
{
    planner.setWorldSize({resolution, resolution});
    planner.setHeuristic(PathPlanner::Heuristic::euclidean);
    planner.setDiagonalMovement(true);
    planner.clearCollisions();
}

TwoBarLinkage::~TwoBarLinkage()
{
}

std::vector<Theta_List> TwoBarLinkage::get_path_c_space(const E_Point &source, const E_Point &target)
{
    std::vector<Theta_List> result;
    if (isReachable(source) && isReachable(target))
    {
        initialize_planner();
        std::vector<Theta_List> solutions = inverseKin(source);
        if(!solutions.size()){
            return result;
        }
        Theta_List source_conf(solutions.back());
        solutions = inverseKin(target);
        if(!solutions.size()){
            return result;
        }
        Theta_List target_conf(solutions.back());

        // angles are scaled to path planner grid resolution
        std::vector<PathPlanner::Vec2i> path(
            planner.findPath({(int)(source_conf[0] * resolution / (2 * M_PI)), (int)(source_conf[1] * resolution / (2 * M_PI))},
                             {(int)(target_conf[0] * resolution / (2 * M_PI)), (int)(target_conf[1] * resolution / (2 * M_PI))}));
        for (auto coordinate = path.rbegin(); coordinate != path.rend(); ++coordinate)
        {
            // scaled back 
            result.push_back(Theta_List{coordinate->x * 2 * M_PI / resolution, coordinate->y * 2 * M_PI / resolution});
        }
    }
    return result;
}

std::vector<Theta_List> TwoBarLinkage::get_path_e_space(const E_Point &source, const E_Point &target)
{
    std::vector<Theta_List> result;
    if (isReachable(source) && isReachable(target))
    {
        initialize_planner();
        add_e_space_obstacle();
        // points are scaled to the path planner grid resolution
        std::vector<PathPlanner::Vec2i> path(
            planner.findPath({(int)((source[0] + l1 + l2) * 0.5 * resolution / (l1 + l2)), (int)((source[1] + l1 + l2) * 0.5 * resolution / (l1 + l2))},
                             {(int)((target[0] + l1 + l2) * 0.5 * resolution / (l1 + l2)), (int)((target[1] + l1 + l2) * 0.5 * resolution / (l1 + l2))}));

        for (auto coordinate = path.rbegin(); coordinate != path.rend(); ++coordinate)
        {
            // scaled back 
            std::vector<Theta_List> conf(inverseKin(E_Point{coordinate->x / (0.5 * resolution / (l1 + l2)) - l1 - l2, coordinate->y / (0.5 * resolution / (l1 + l2)) - l1 - l2}));
            if(!conf.size()){
                return {};
            }
            result.push_back(conf.back());
        }
    }
    return result;
}

bool TwoBarLinkage::isReachable(const E_Point &point)
{
    double distance(sqrt(sq(point[0]) + sq(point[1])));
    return l1 - l2 <= distance && distance <= l1 + l2;
}

E_Point TwoBarLinkage::forwardKin(const Theta_List &thetalist)
{
    E_Point point({l1 * cos(thetalist[0]) + l2 * cos(thetalist[0] + thetalist[1]), l1 * sin(thetalist[0]) + l2 * sin(thetalist[0] + thetalist[1])});
    return point;
}

double TwoBarLinkage::absoluteAngle(double angle)
{
    return (angle > 0) ? angle : 2 * M_PI + angle;
}

std::vector<std::array<double, 2>> TwoBarLinkage::inverseKin(const E_Point &point)
{
    std::vector<std::array<double, 2>> solutions;
    if (!isReachable(point))
    {
        return solutions;
    }
    double alpha, beta, gamma, distance_sq, theta1, theta2;
    distance_sq = sq(point[0]) + sq(point[1]);
    alpha = acos((distance_sq + sq(l1) - sq(l2)) / (2 * l1 * sqrt(distance_sq)));
    beta = acos((sq(l1) + sq(l2) - sq(point[0]) - sq(point[1])) / (2 * l1 * l2));
    if(!std::isnan(alpha) && !std::isnan(beta))
    {
        gamma = atan2(point[1], point[0]);
        solutions.push_back(std::array<double, 2>{absoluteAngle(gamma - alpha), absoluteAngle(M_PI - beta)});
        solutions.push_back(std::array<double, 2>{absoluteAngle(gamma + alpha), absoluteAngle(beta - M_PI)});
    }
    return solutions;
}

void TwoBarLinkage::add_line_obstacle(E_Point start, E_Point end)
{
    obstacle = {start, end};
}

void TwoBarLinkage::remove_obstacle()
{
    obstacle = {};
}

void TwoBarLinkage::add_e_space_obstacle()
{
    Line obs_line(obstacle[0], obstacle[1]);
    E_Point point = (obstacle[0][0] < obstacle[1][0]) ? obstacle[0] : obstacle[1];
    while (true)
    {
        if (isReachable(point))
        {
            PathPlanner::Vec2i obs;
            obs.x = (int)((point[0] + l1 + l2) * 0.5 * resolution / (l1 + l2));
            obs.y = (int)((point[1] + l1 + l2) * 0.5 * resolution / (l1 + l2));
            planner.addCollision(obs);
        }
        if (point[0] > obstacle[1][0])
        {
            break;
        }
        point[0] += 2 * (l1 + l2) / resolution;
        point[1] = obs_line.y(point[0]);
    }
}

void TwoBarLinkage::add_c_space_obstacle()
{
    Line obs_line(obstacle[0], obstacle[1]);
    E_Point point = (obstacle[0][0] < obstacle[1][0]) ? obstacle[0] : obstacle[1];
    while (true)
    {
        if (isReachable(point))
        {
            PathPlanner::Vec2i obs;
            Theta_List conf = inverseKin(point).back();
            obs.x = (int)(conf[0] * resolution / (2 * M_PI));
            obs.y = (int)(conf[1] * resolution / (2 * M_PI));
            planner.addCollision(obs);
        }
        if (point[0] > obstacle[1][0])
        {
            break;
        }
        point[0] += 2 * (l1 + l2) / resolution;
        point[1] = obs_line.y(point[0]);
    }
}