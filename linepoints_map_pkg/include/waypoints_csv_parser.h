//
//  pid_control.h
//  pcan_mac
//
//  Created by Changhyeon Park on 16/08/2018.
//

#ifndef WAYPOINTS_INFO_STRUCT
#define WAYPOINTS_INFO_STRUCT
#include <iostream>
#include <vector>

typedef struct WaypointsInfo
{
    double latitude;
    double longitude;
    double x;
    double y;
    double angle;
    double distance;
    int parent_node_nID;
}WPI;
#endif /* WAYPOINTS_INFO_STRUCT */

#ifndef WAYPOINTS_SCV_PARSER_h
#define WAYPOINTS_SCV_PARSER_h
/*
 * Waypoints Definition
 */
#define     __ID__              0
#define     __L_LINK_ID__       9
#define     __R_LINK_ID__       8
#define     __F_NODE_ID__       10
#define     __T_NODE_ID__       11
#define     __LENGTH__          13
#define     __MAX_SPEED__       6
#define     __WAYPOINTS__       23
#define     __LONGITUDE__       26
#define     __LATITUDE__        27
#define     __ANGLE__           25
#define     __DISTANCE__        24
#define     __ROAD_RANK__       2
#define     __ROAD_TYPE__       3
#define     __ROAD_NO__         4       // Not good
#define     __LINK_TYPE__       5       // Intersection : 1, Etc : 6
#define     __ITS_LINK_ID__     14
#define     __SECTION_ID__      12
#define     __NULL__            -1

//typedef struct WaypointsInfo
//{
//    double latitude;
//    double longitude;
//    double x;
//    double y;
//    double angle;
//    double distance;
//}WPI;

template<typename T>
struct GlobalWaypointsInfo
{
    std::string id;
    int nID;
    T l_link_nID;
    T r_link_nID;
    std::vector<T> f_node_nID;
    std::vector<T> t_node_nID;
    std::vector<T> l_line_nID;
    std::vector<T> r_line_nID;
    int lane_no;
    double length;
    double max_speed;
    int link_type;

    double cost;
    int prev_node;
    bool closed_flag;

    std::vector<WaypointsInfo> wp;
};

#endif /* WAYPOINTS_SCV_PARSER_h */
