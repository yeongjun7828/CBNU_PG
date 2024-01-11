//
//  pid_control.h
//  pcan_mac
//
//  Created by Changhyeon Park on 16/08/2018.
//
#ifndef WAYPOINTS_INFO_STRUCT
#define WAYPOINTS_INFO_STRUCT
#include "iostream"
#include <vector>

typedef struct WaypointsInfo
{
    double latitude;
    double longitude;
    double x;
    double y;
    double angle;
    double distance;
}WPI;
#endif /* WAYPOINTS_INFO_STRUCT */

#ifndef LINES_CSV_PARSER_h
#define LINES_CSV_PARSER_h
/*
 * Line Points Definition
 */
#define     __LINE_ID__         0
#define     __LINE_TYPE__       2
#define     __LINE_KIND__       3
#define     __LINE_R_LINK_ID__  4
#define     __LINE_L_LINK_ID__  5
#define     __LINE_POINTS__     14
#define     __LINE_DISTANCE__   15
#define     __LINE_ANGLE__      16
#define     __LINE_LONGITUDE__  17
#define     __LINE_LATITUDE__   18

template<typename T>
struct GlobalLinesInfo
{
    std::string id;
    int nID;
    int type;
    int kind;
    T l_link_nID;
    T r_link_nID;

    std::vector<WaypointsInfo> wp;
};

#endif /* LINES_CSV_PARSER_h */
