#ifndef UTILS_H
#define UTILS_H

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

/**
 * @brief timeDiff Computes the time difference between two time intervals, in microseconds
 * @param start start time
 * @param end   end time
 * @return
 */
double timeDiff(struct timeval start, struct timeval end);

/**
 * @brief normalizeAngle Normalizes the angle to be in [-pi, pi)
 * @param x input angle [rad]
 * @return normalized angle [rad]
 */
double normalizeAngle(double x);

/**
 * @brief sign Computes the sign of a number
 * @param x input number
 * @return the sign
 */
int sign(double x);

bool insideInterval(double start, double end, double x);

double distance(Position_t p1, Position_t p2);


#endif // UTILS_H
