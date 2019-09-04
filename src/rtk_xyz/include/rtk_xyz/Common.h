#include <iostream>
#include <ros/ros.h>

#define R0 6378137.0
#define e 0.0818191908425

#define L0 32.2056821
#define lamda0 119.3719783

#define hb 0


struct positionConf
{
    u_int32_t n_gps_sequence_num;
    double x;
    double y;
    double z;
    double lon;
    double lat;
    double height;
    double velocity_x;
    double velocity_y;
    double velocity_z;
    double heading;
    double pitch;
    double roll;
    double gps_seconds;
    double velocity;
    double dist;
};


void gps2xy(positionConf &xy_p, const positionConf &real_p);
