#ifndef MYTYPES_H
#define MYTYPES_H

#include "math.h"

#ifndef DEG2RAD
#define DEG2RAD M_PI/180.0
#endif
/**
 * @brief The Position_t struct Represents a 2D position, in meters
 */
typedef struct
{
    double x; // [m]
    double y; // [m]
}Position_t;


typedef struct
{
    double x;       // [m]
    double y;       // [m]
    double theta;   // [rad]
}Pose2D_t;

#endif // MYTYPES_H
