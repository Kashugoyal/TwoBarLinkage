/*
    This file defines a Line Class
    Author: Kashish Goyal
*/

#ifndef LINE_HPP
#define LINE_HPP

typedef std::array<double, 2> E_Point; // euclidian point
#include "TwoBarLinkage.hpp"

/*
    Line
    Helper class to get points on a line, header only
*/
class Line
{
public:

    /*
        Line Constructor
        @params: 
            E_Point: first point on the line, 
            E_Point: second point on the line, 
    */
    Line(E_Point point1, E_Point point2)
    {
        a = point1[1] - point2[1];
        b = point2[0] - point1[0];
        c = point1[0]*point2[1] - point1[1]*point2[0];
    }

    virtual ~Line(){}

    /*
        Function to get the y cordinate of a point on the line corresponding to the given x coordinate
        @params: 
            double: x coordinate of the point
        @return:
            double: y coordinate of the point on the line
    */
    inline double y(double x)
    {
        return (-c - a*x)/b;
    }

private:
    double a; //x co-efficient
    double b; // y co-efficient
    double c; // intercept

};

#endif //LINE_HPP