/**
  Vector Field Histogram Star algorithm for local obstacle avoidance with look-ahead.

  VFH*: Local Obstacle Avoidance with Look-Ahead Verification
  */
#ifndef VFH_H
#define VFH_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <urglib2.h>
#include <string.h>
#include <vector>

#include <urglib2.h>
#include "myTypes.h"
#include "utils.h"
#include "vector2.h"

#define MIN_OBSTACLE_DISTANCE 0.5 // Min allowed distance from robot to obstacle, in m
#define SEC_MARGIN            0.25 // Security margin

// Laser Scanner params
#define URG_RES         0.25*M_PI/180 // Resolution of the laser scanner: 0.25º (in rad)
#define URG_RANGE       270*M_PI/180  // Range of the laser scanner: 270º (in rad)

// VFH tuning params
#define VFH_ALPHA       10.0
#define VFH_BETA        0.001 //Since the scan is given in mm

// PD controller gains
#define CONTROLLER_KP 1.5
#define CONTROLLER_KD 1.0

typedef struct
{
    double startAngle;
    double endAngle;
}Opening_t;


class VFH
{
public:
    VFH();

    void computeCommands(Pose2D_t *robotPose, Position_t *goal, tScan *scan, double *v, double *w);

private:

    // PD controller
    double prevError_;

    int analyzeScan(tScan* scan, double targetTheta, std::vector<double> &histogram);
    double computeRobotSteering(std::vector<double> &histogram);
    void computeRobotCommands(double currentTheta, double targetTheta, double* v, double *w);
    double getAngleFromIndex(int i);
    double refineAngle(bool start, Vector2* pHit);
};

#endif // VFH_H
