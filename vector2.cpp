#include "vector2.h"

Vector2::Vector2()
{
    x_ = 0;
    y_ = 0;
}

Vector2::Vector2(double x, double y)
{
    x_ = x;
    y_ = y;
}

static Vector2 add(Vector2 v1, Vector2 v2)
{
    Vector2 v(v1.x_ + v2.x_, v1.y_ + v2.y_);
    return v;
}

static Vector2 subtract(Vector2 v1, Vector2 v2)
{
    Vector2 v(v1.x_ - v2.x_, v1.y_ - v2.y_);
    return v;
}

void Vector2::normalize()
{
    double mod = sqrt(x_*x_ + y_*y_);
    x_/=mod;
    y_/=mod;
}

bool Vector2::isZero()
{
    return (x_ == 0 && y_ == 0);
}
