#include "vfh.h"

VFH::VFH()
{
    prevError_ = 0.0;
}

void VFH::computeCommands(Pose2D_t* robotPose, Position_t* goal, tScan* scan, double* v, double* w)
{
    // **** Get desired theta orientation towards the goal
    double targetTheta = atan2(goal->y - robotPose->y, goal->x - robotPose->x);
    printf("Desired theta: %.3f\n", targetTheta*RAD2DEG);

    // **** Analize scan to get available directions
    std::vector<double> histogram;
    int nOpenings = analyzeScan(scan, targetTheta, histogram);
    printf("Number of openings: %d\n", nOpenings);

    // **** Get direction that is closest to goal
    double commandTheta;
    if(nOpenings > 0)
    {
        commandTheta = computeRobotSteering(histogram); //Global coordinates
        printf("Command theta (local): %.3f\n", commandTheta*RAD2DEG);
        commandTheta = commandTheta + robotPose->theta;
        printf("Command theta (global): %.3f\n", commandTheta*RAD2DEG);
    }
    else // No opening found => rotate 180º to find other path
    {
        commandTheta = normalizeAngle(robotPose->theta + M_PI);
    }

    // **** Compute robot commands
    computeRobotCommands(robotPose->theta, commandTheta, v, w);
    printf("Robot commands: %.3f, %3f\n", *v, *w);
}


int VFH::analyzeScan(tScan* scan, double targetTheta, std::vector<double> &histogram)
{
    bool onOpening = false;
    double startAngle = 0.0, endAngle = 0.0;
    int nValleys = 0;
    std::vector<Opening_t> openings;
    openings.clear();
    bool collision=false;

    Vector2 prevHitPoint;
    // **** Analyze laser scan (from +135º to -135º)
    for(unsigned int i=0; i<scan->size;i++)
    {
        double angle = getAngleFromIndex(i);           // Angle of the ray (rad)
        double depth = (scan->rawScan[i])/1000.0;      // Depth (m)
        collision = depth < MIN_OBSTACLE_DISTANCE;

        // Check if the obstacle far enough
        if (!collision && i < scan->size-1) //Skip the last ray as ending angle
        {
            if(!onOpening)
            {
                onOpening = true;
                startAngle = angle;

                // Refine angle
                if(!prevHitPoint.isZero()) //No need to refine angle if there is no previous hit
                    startAngle = refineAngle(true, &prevHitPoint);
            }
        }
        else
        {
            if(collision)
            {
                // Update the collision point, in LOCAL coordinate frame
                prevHitPoint.x_ = scan->pts->v[0];
                prevHitPoint.y_ = scan->pts->v[1];
            }

            if(onOpening) //Finish the opening
            {
                onOpening = false;
                // Refine angle if this is not the last ray
                if(i != scan->size-1)
                {
                    endAngle = refineAngle(false, &prevHitPoint);
                }
                else
                    endAngle = angle;

                // Add the opening to the vector
                if ((endAngle - startAngle) > 0)
                {
                    Opening_t open;
                    open.startAngle = startAngle;
                    open.endAngle = endAngle;
                    openings.push_back(open);
                    printf("Opening %zu. Start angle: %.3f, End angle: %.3f\n", openings.size(), open.startAngle*RAD2DEG, open.endAngle*RAD2DEG);
                }
            }
        }
    }    

    // **** Fill in the histogram using the openings
    for (int i=0;i < scan->size; i++)
    {
        double angle = getAngleFromIndex(i);

        for(int j=0;j<openings.size();j++)
        {
            Opening_t open = openings.at(j);
            if(insideInterval(open.startAngle,open.endAngle,angle))
            {
                histogram[i] = VFH_ALPHA*1.0/fabs(targetTheta-angle) + VFH_BETA*scan->rawScan[i];
                break;
            }
            else{ // Not a good region => negative weight
                histogram[i] = -1;
            }
        }

    }
    return openings.size();
}

double VFH::refineAngle(bool start, Vector2* pHit)
{
    // **** Get unitary vectors perpendicular to the hit vector
    Vector2 pHitPerp1(pHit->y_, -pHit->x_);
    Vector2 pHitPerp2(-pHit->y_, pHit->x_);
    pHitPerp1.normalize();
    pHitPerp2.normalize();

    // **** Get new vectors at a security margin distance from the hit
    Vector2 newPHit1 = Vector2(pHit->x_ + SEC_MARGIN * pHitPerp1.x_, pHit->y_ + SEC_MARGIN * pHitPerp1.y_);
    Vector2 newPHit2 = Vector2(pHit->x_ + SEC_MARGIN * pHitPerp2.x_, pHit->y_ + SEC_MARGIN * pHitPerp2.y_);

    // **** Get angle of these vectors (in laser coordinate frame, -sign)
    double newAngle1 = atan2(newPHit1.y_, newPHit1.x_); // X-axis up, Y-axis right
    double newAngle2 = atan2(newPHit2.y_, newPHit2.x_);

    // **** Select the most restrictive one, depending on whether it's start or end angle
    if(start)
        return std::max(newAngle1, newAngle2);
    else
        return std::min(newAngle1, newAngle2);
}

double VFH::computeRobotSteering(std::vector<double> &histogram)
{
    /// @todo Foresee deadlocks (VFH*)


    // **** Analyze directions and take the best one
    double weight, maxWeight = -10;
    double targetTheta;

    for(unsigned int i=0; i<histogram.size(); i++)
    {
        weight = histogram[i];

        if (weight > maxWeight)
        {
            maxWeight = weight;
            targetTheta = getAngleFromIndex(i);
        }
    }
    return targetTheta;
}

void VFH::computeRobotCommands(double currentTheta, double targetTheta, double* v, double *w)
{
    /// @todo implement speed control based on proximity to obstacles

    // **** PD controller
    double error = targetTheta - currentTheta;

    *w = CONTROLLER_KP * error + CONTROLLER_KD * prevError_;
    *v = 0.0; // CHANGE THIS!!

    // Update variable for PD
    prevError_ = error;
}

double VFH::getAngleFromIndex(int i)
{
    return -URG_RANGE/2.0 + (i*URG_RES);
}
