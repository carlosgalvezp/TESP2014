#ifndef VECTOR2_H
#define VECTOR2_H

#include "math.h"

class Vector2
{
public:
    double x_, y_;

    Vector2();
    Vector2(double x, double y);
    Vector2 add(Vector2 v1, Vector2 v2);
    Vector2 subtract(Vector2 v1, Vector2 v2);
    void normalize();
    bool isZero();
};

#endif // VECTOR2_H
