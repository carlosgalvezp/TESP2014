#include "utils.h"

double timeDiff(struct timeval start, struct timeval end)
{
    return (end.tv_sec - start.tv_sec)*1000000.0 + (end.tv_usec - start.tv_usec);
}

int sign(double x)
{
    if (x >= 0)
        return 1;
    else
        return -1;
}

double normalizeAngle(double x)
{
    if (fabs(x) > M_PI)
        x = x - (2.0 * M_PI * sign(x));

    return x;
}

bool insideInterval(double start, double end, double x)
{
    return x > start && x < end;
}

double distance(Position_t p1, Position_t p2)
{
    return fabs(p1.x - p2.x) + fabs(p1.y - p2.y);
}
