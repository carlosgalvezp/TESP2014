#include "main.h"

int main(int argc, char *argv[])
{
    // Grab input parameters
    if (argc <2)
    {
        printf("Usage: ./main option");
        printf("Ex: ./main 1");
        printf("----------------------------------------");
        printf("Options: ");
        printf("1) Test Pose");
        printf("2) Test URG");
        printf("3) Test distance keeping");
        printf("4) Main program, autonomous navigation");
        return -1;
    }

    int mode = atoi(argv[1]);

    // Initialize
    initSensors();

    // Run selected program

    switch (mode)
    {
        case 1:
            testPose();
            break;
        case 2:
            testURG();
            break;
        case 3:
            testDistanceKeep(MIN_OBSTACLE_DISTANCE);
            break;
        case 4:
            Position_t goal;
            goal.x = 1.0;
            goal.y = 1.0;
            moveToGoal(goal);
            break;
    }

    // When done, disconnect
    disconnect();
}

void testPose()
{
    timeval t1,t2;
    double deltaT;

    // Small movement for the robot
    porco::cloco::set_vw(0,0); // cm/s, deg/s

    while(doIt)
    {
        // **** Get pose
        gettimeofday(&t1, NULL);
        porco::cloco::get_pos(&robotPose_.x,&robotPose_.y,&robotPose_.theta);
        gettimeofday(&t2,NULL);

        deltaT = timeDiff(t1,t2);

        // **** Display
        printf("[%lu.%lu] X: %.4f [m], Y: %.4f [m], Theta: %.4f [rad] (%.4f ms)\n",
               t2.tv_sec,t2.tv_usec, robotPose_.x, robotPose_.y, robotPose_.theta, deltaT/1000.0);

        // **** Wait for timer
        usleep(std::max(0.0, SAMPLE_TIME*1000000.0 - deltaT));
    }
}

void testURG()
{
    printf("Entering testURG...\n");
    timeval t1,t2;
    int j;
    int temp=1;
    double deltaT;

    FILE* output = fopen(urgLogFile, "w"); // To save the data
    assert(output != NULL);

    tScan* scan;

    while(doIt)
    {
        // **** Get laser scaner in cartesian coordinates
        gettimeofday(&t1, NULL);
        getLaserScan(scan, true);
        gettimeofday(&t2,NULL);

        deltaT = (t2.tv_sec - t1.tv_sec)*1000000.0 - (t2.tv_usec - t1.tv_usec);

        temp=fprintf(output, "[%lu.%lu]---------------------------------\n", \
		t2.tv_sec,t2.tv_usec);

        printf("[%lu.%lu] URG Scan: %d points, %.4f ms\n", \
	       t2.tv_sec,t2.tv_usec, scan->size, (t2.tv_sec - t1.tv_sec)*1000 + (t2.tv_usec - t1.tv_usec)/1000.0);
        assert(temp>0);
 
        // **** Save to file
        for(j=0;j<scan->size;j++)
        {
             temp=fprintf(output,"%d %f %f\n",
                         scan->rawScan[j],
                         scan->pts[j].v[0],
                         scan->pts[j].v[1]); 
	     assert(temp>0);
        }

        // **** Wait for timer
        sleep(std::max(0.0, SAMPLE_TIME*1000000.0 - deltaT));
    }
}

void testDistanceKeep(double distance)
{
    printf("Entering testDistanceKeep...\n");
    timeval t1,t2;
    int j;
    double minDistance, deltaT;

    tScan* scan;

    while(doIt)
    {
        minDistance = 100000.0;
        // **** Get laser scaner in cartesian coordinates
        gettimeofday(&t1, NULL);
        getLaserScan(scan, true);
        gettimeofday(&t2,NULL);

        deltaT = (t2.tv_sec - t1.tv_sec)*1000000.0 - (t2.tv_usec - t1.tv_usec);

        printf("[%lu.%lu] URG Scan: %d points, %.4f ms\n", \
                t2.tv_sec,t2.tv_usec, scan->size, (t2.tv_sec - t1.tv_sec)*1000 + (t2.tv_usec - t1.tv_usec)/1000.0);

        // **** Analyze laser scan
        for(j=0;j<scan->size;j++)
        {
            if (scan->rawScan[j] < minDistance)
                minDistance = scan->rawScan[j];
        }

        if(minDistance < distance)
        {
            porco::cloco::stop();
            break;
        }
        else
            porco::cloco::set_vw(10,0);

        printf("Min distance to object: %.4f mm", minDistance);

        // **** Wait
        sleep(std::max(0.0, SAMPLE_TIME*1000000.0 - deltaT));
    }
}

void moveToGoal(Position_t goal)
{
    bool arrived = false;
    struct timeval t0, t1, t2;
    gettimeofday(&t0, NULL);

    double tPose, tLaser, tVFH;
    double v, w; // Robot commands
    tScan scan;
    VFH vfh;
    porco::cloco::set_pos(0,0,0);
    while(doIt && !arrived)
    {
        // **** Update robot pose       
        gettimeofday(&t0, NULL);
            porco::cloco::get_pos(&robotPose_.x,&robotPose_.y,&robotPose_.theta);
            // Transform into correct units
            robotPose_.x /= 100.0; // It was given in cm/s
            robotPose_.y /= 100.0; // It was given in cm/s
            robotPose_.theta *= DEG2RAD; // It was given in deg
        gettimeofday(&t2,NULL);
        tPose = timeDiff(t0,t2);

        // **** Get laser scan
        gettimeofday(&t1, NULL);
            getLaserScan(&scan, true);
        gettimeofday(&t2,NULL);
        tLaser = timeDiff(t1,t2);
        printf("Number of points scan: %d\n", scan.size);

        // **** Compute input commands (using path planner or VFH)
        printf("Computing robot commands\n");
        gettimeofday(&t1, NULL);
            vfh.computeCommands(&robotPose_, &goal, &scan, &v, &w);
        gettimeofday(&t2,NULL);
        tVFH = timeDiff(t1,t2);

        // **** Set robot velocity
            porco::cloco::set_vw(v,w);

        // **** Print information
        printf("[%lu.%lu] Pose: [%.3f, %.3f, %.3f] Time: [%.3f, %.3f, %.3f] Total: %.3f\n",
               t2.tv_sec, t2.tv_usec, robotPose_.x, robotPose_.y, robotPose_.theta,
               tPose/1000.0, tLaser/1000.0, tVFH/1000.0, (tPose + tLaser + tVFH)/1000.0);


        // **** Check if arrived to goal
        Position_t robotP;
        robotP.x = robotPose_.x;
        robotP.y = robotPose_.y;

        arrived = (distance(robotP, goal) < EPSILON_DISTANCE);
        // **** Wait for timer
        gettimeofday(&t2, NULL);
        //usleep(SAMPLE_TIME*1000000 - timeDiff(t0,t2));
        doIt = false;
    }
}

void autonomousNavigation()
{
    // **** Move to a sequence of goals
}


void initSensors()
{
    // **** Initialize variables
    robotPose_.x = 0.0;
    robotPose_.y = 0.0;
    robotPose_.theta = 0.0;

    // **** Initialize CAN bus
    porco::can::init();
    porco::cloco::init();
    unsigned char odomode=4;
    bool wc_res;
    wc_res = porco::can::write_control(1,CONTROL_CHANGE_MODE,1,&odomode);// 3D loco mode
    if(!wc_res){
      printf("write_control error\n");
    }

    // Signal handler for SIGINT
    signal(SIGINT,sigIntHandler);


    // Initialize URG laser scaner
    // open and get information from urg
    // aDelay can be estimated to 1 ms if the URG is connected via USB.
    urg=urgOpen(urgAddress,1);
    if(urg==NULL)
    {
        exit(1);
    }
    printf(" %s opened \n",urgAddress);


    printf("URG parameters\n");
    printf(" Model Name\t: %s\n",urg->modelStr);
    printf(" Min dist\t: %d[mm]\n",urg->minDist);
    printf(" Max dist\t: %d[mm]\n",urg->maxDist);
    printf(" Min step\t: %d\n",urg->minStep);
    printf(" Max step\t: %d\n",urg->maxStep);
    printf(" Angle Res\t: %d\n",urg->angRes);
    printf(" Front step\t: %d\n",urg->frontStep);
    printf(" Max scan Speed\t: %d[rpm]\n",urg->scanSpeed);
    {
      char buff[64];
      printf(" Epoc\t: %s , %03ld[ms]\n",
             ctime_r(&urg->epoc.tv_sec,buff),urg->epoc.tv_usec/1000);
    }
    urgStopAll(urg);
    urgReset(urg);


    // 1 scan per sec(skip 9scans), all point per scan, infinite scans
    printf("urgStartScan :%d\n",urgStartScan(urg,urg->minStep,urg->maxStep,1,9,0,URG_3BYTE_ENC));

    // **** Initialize theta
    porco::cloco::set_pos(robotPose_.x, robotPose_.y, robotPose_.theta);
}

void disconnect()
{
    printf("Stopping and disconnecting...");
    // Stop the robot
    porco::cloco::stop();

    // Disconnect the URG sensor
    urgFinalizeUrg(urg);
}


void getLaserScan(tScan* scan, bool cartesian)
{
    *scan=*(urgFetchScan(urg)); //Scan: rawScan[i] | pts[j].v[0] | pts[j].v[1]

    if(scan==NULL)
    { // shuld not be reached
      fprintf(stderr,"**DEBUG**: : FetchScan failed\n");
    }
    else
    {
      if (cartesian) urgPolar2Cart(urg,scan);      
    }
}

void computeControlInput(double *v, double *w)
{
    /// @todo
}

void sigIntHandler(int sig)
{
    doIt = 0;
}



