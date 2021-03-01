#ifndef LINE_HPP
#define LINE_HPP

typedef std::array<double, 2> E_Point; // euclidian point
#include "TwoBarLinkage.hpp"

class Line
{
public:
    Line(E_Point point1, E_Point point2)
    {
        a = point1[1] - point2[1];
        b = point2[0] - point1[0];
        c = point1[0]*point2[1] - point1[1]*point2[0];
    }
    virtual ~Line(){}
    inline double y(double x)
    {
        return (-c - a*x)/b;
    }

private:
    double a; //x
    double b; // y
    double c; // intercept

};

#endif //LINE_HPP