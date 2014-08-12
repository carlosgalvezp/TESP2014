#ifndef MAIN_H
#define MAIN_H

#include <regmapReader.hh>
#include <porco_can_loco_client.hpp>
#include <porco_can_client.hpp>
#include <stdio.h>
#include <ncurses.h>
#include <sys/types.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include <sys_controlMsg.h>
#include <unistd.h>
#include <assert.h>

#include <urglib2.h>
#include <string.h>
#include <signal.h>

#include "myTypes.h"
#include "utils.h"
#include "vfh.h"

#define SAMPLE_TIME 0.5 // Sample time, in seconds
#define EPSILON_DISTANCE 0.1 // In meters
/**
 * @brief initSensors Initializes the system (CAN bus and URG laser scan)
 */
void initSensors();

/**
 * @brief disconnect Stops the robot and disconnects the sensors
 */
void disconnect();

/**
 * @brief getLaserScan Makes one measurement of the laser scan
 * @param scan scan object
 * @param cartesian True if convert the output to cartesian coordinates, in robot frame
 */
void getLaserScan(tScan *scan, bool cartesian);

/**
 * @brief computeControlInput Computes the commands for the robot (linear and angular velocity)
 * @param v linear velocity [m/s]
 * @param w angular velocity [rad/s]
 */
void computeControlInput(double *v, double *w);

/**
 * @brief sigIntHandler Function to be called when SIGINT appears (Ctrl-C)
 * @param sig
 */
void sigIntHandler(int sig);

/**
 * @brief testPose Gets the pose of the robot (x,y,theta) and displays it
 */
void testPose();

/**
 * @brief testURG Gets laser scanner measurements and saves them into a file
 */
void testURG();

/**
 * @brief testDistanceKeep Moves the robot forward until it detects an object at a given distance
 * @param distance minimum distance [m]
 */
void testDistanceKeep(double distance);

/**
 * @brief moveToGoal Moves the robot from its current position towards the desired goal
 * @param goal the desired goal
 */
void moveToGoal(Position_t goal);

/**
 * @brief autonomousNavigation The robot moves autonomously from start to goal, avoiding obstacles
 */
void autonomousNavigation();

// ************************************************************************************
// ************************************************************************************
char* urgLogFile = "scan.log";
char* urgAddress = "/dev/ttyACM0";

// Main variables
int doIt = 1;

// Laser scanner variables
tURG *urg;
FILE *output;

// Robot pose, in world coordinates
Pose2D_t robotPose_;

// ************************************************************************************
// ************************************************************************************

#endif // MAIN_H
