#include "ros/ros.h" // ROS 기본 헤더파일
#include <iostream>
#include <fstream>
#include "pcl_ros/point_cloud.h"
#include <pcl/point_cloud.h>
#include "waypoints_csv_parser.h"
#include "lines_csv_parser.h"

#include <UTM.h>

typedef GlobalWaypointsInfo<std::string>    StrGWPI;
typedef GlobalWaypointsInfo<double>         DoubGWPI;
typedef GlobalLinesInfo<std::string>        StrGLI;
typedef GlobalLinesInfo<double>             DoubGLI;

std::vector<std::string> csv_read_row(std::istream &in, char delimiter);
std::vector<std::string> csv_read_row(std::string &line, char delimiter);

void GetInfoTreeMap(std::vector<DoubGWPI> &gwpi, std::vector<DoubGLI> &gli, const std::pair<double, double> utm_xy_offset_);
void WayPointsInterpolating(std::vector<WaypointsInfo> &global_waypoints, const double inter_dist_between_wps/* Unit: meter*/);
void RecurCheckLaneNo(const std::vector<DoubGWPI> gwpi_, const int id_, int &lane_no_);
std::vector<DoubGLI> GetLineInfoTreeMap(const std::vector<DoubGWPI> gwpi, const std::pair<double, double> utm_xy_offset_);



int main(int argc, char **argv)
{
    ros::init(argc, argv, "linepoints_map_node"); // 노드명 초기화


    ros::NodeHandle nh_rviz;
    ros::Publisher global_map_pcl_pub = nh_rviz.advertise<sensor_msgs::PointCloud2>("global_map_lines", 1);

    ros::NodeHandle nh_get_frame_id;
    std::string frame_id;
    std::string param_name = "/linepoints_map/frame_id";
    nh_get_frame_id.getParam(param_name, frame_id);

    std::vector<DoubGWPI> gwpi;
    std::vector<DoubGLI> gli;
    double ego_latitude_offset = 37.5800;      // deg
    double ego_longitude_offset = 126.8900;    // deg
    double ego_utm_x_offset = 313663;
    double ego_utm_y_offset = 4160770;
    nh_get_frame_id.getParam("/linepoints_map/UTM_Offset/X", ego_utm_x_offset);
    nh_get_frame_id.getParam("/linepoints_map/UTM_Offset/Y", ego_utm_y_offset);

//    LatLonToUTMXY(ego_latitude_offset, ego_longitude_offset, 0, ego_utm_x_offset, ego_utm_y_offset);

    std::pair<double, double> utm_xy_offset = std::make_pair(ego_utm_x_offset, ego_utm_y_offset);
    GetInfoTreeMap(gwpi, gli, utm_xy_offset);

    /* Interpolating */
    double inter_dist_between_wps = 1; // Unit : meter
    for(int i = 0; i < gwpi.size(); i++)
        for(int j = 0; j < gwpi[j].wp.size(); j++)
            WayPointsInterpolating(gwpi[i].wp, inter_dist_between_wps);
    // Global Linepoints 에 Interpolatiing 작업.
    for(int i = 0; i < gli.size(); i++)
        for(int j = 0; j < gli[i].wp.size(); j++)
            WayPointsInterpolating(gli[i].wp, inter_dist_between_wps);


    std::vector<WaypointsInfo> entire_linepoints;
    for(int i = 0; i < gli.size(); i++)
        for(int j = 0; j < gli[i].wp.size(); j++)
        {
            entire_linepoints.push_back({gli[i].wp[j].latitude,
                                        gli[i].wp[j].longitude,
                                        gli[i].wp[j].x,
                                        gli[i].wp[j].y,
                                         gli[i].wp[j].angle,
                                         gli[i].wp[j].distance,
                                         gli[i].wp[j].parent_node_nID});
        }


    while(ros::ok())
    {
        //////////////////////////////////////
        // Entire Map Linepoints Visualizer //
        //////////////////////////////////////
        std::vector<std::pair<double, double>> line_points_xy;
        for(int i = 0; i < entire_linepoints.size(); i++)
        {
            std::pair<double, double> line_points_xy_tmp = std::make_pair(entire_linepoints[i].x, entire_linepoints[i].y);
//            double map_theta_offset = -1.78; // deg
//            std::pair<double, double> line_points_xy_tmp = CoordConvert_GlobalToVehicle(std::make_pair(entire_linepoints[i].x, entire_linepoints[i].y), {0, 0}, (90+map_theta_offset) * M_PI / 180.);
            line_points_xy.push_back(line_points_xy_tmp);
        }
        pcl::PointCloud<pcl::PointXYZ> store2;
        for (int i = 0; i < line_points_xy.size(); i++)
        {
            pcl::PointXYZ pos_tmp;
            pos_tmp.x = line_points_xy[i].first;
            pos_tmp.y = line_points_xy[i].second;
            pos_tmp.z = -0.1;
            store2.push_back(pos_tmp);
        }
        store2.header.frame_id = frame_id;
        global_map_pcl_pub.publish(store2);

        usleep(1000);

    }

    return 0;
}

void GetInfoTreeMap(std::vector<DoubGWPI> &gwpi, std::vector<DoubGLI> &gli, const std::pair<double, double> utm_xy_offset_)
{
    ros::NodeHandle nh_get_global_path_file;
    std::string file_dir;
    std::string param_name = "/linepoints_map/EntirePathFile";
    nh_get_global_path_file.getParam(param_name, file_dir);

    std::vector<StrGWPI> gwpi_tmp;
    std::ifstream fin_tmp(file_dir);
    std::string cell_tmp;
    std::string line_tmp;
    bool skip_flag = true;
    int nID_tmp = 0;
    while(fin_tmp.good())
    {
        std::vector<std::string> row = csv_read_row(fin_tmp, ',');

        // csv_read_row 다음에 나와야함.
        if(skip_flag == true)
        {
            skip_flag = false;
            continue;
        }
        std::pair<double, double> utm_xy_tmp;
        LatLonToUTMXY(std::stod(row[__LATITUDE__]), std::stod(row[__LONGITUDE__]), 0, utm_xy_tmp.first, utm_xy_tmp.second);

        WPI wpi_tmp;
        wpi_tmp.latitude = std::stod(row[__LATITUDE__]);
        wpi_tmp.longitude = std::stod(row[__LONGITUDE__]);
        wpi_tmp.x = utm_xy_tmp.first - utm_xy_offset_.first;
        wpi_tmp.y = utm_xy_tmp.second - utm_xy_offset_.second;
        wpi_tmp.angle = std::stod(row[__ANGLE__]);
        wpi_tmp.distance = std::stod(row[__DISTANCE__]);
        wpi_tmp.parent_node_nID = __NULL__;

        StrGWPI gwpi_tmp_2;
        gwpi_tmp_2.id = row[__ID__];
//        gwpi_tmp_2.nID = nID_tmp++;
        gwpi_tmp_2.l_link_nID = row[__L_LINK_ID__];
        gwpi_tmp_2.r_link_nID = row[__R_LINK_ID__];
        gwpi_tmp_2.f_node_nID.push_back(row[__F_NODE_ID__]);
        gwpi_tmp_2.t_node_nID.push_back(row[__T_NODE_ID__]);
        gwpi_tmp_2.l_line_nID.clear();
        gwpi_tmp_2.r_line_nID.clear();
        gwpi_tmp_2.lane_no = __NULL__;
        gwpi_tmp_2.length = std::stod(row[__LENGTH__]);
        gwpi_tmp_2.max_speed= std::stod(row[__MAX_SPEED__]);
        gwpi_tmp_2.link_type = std::stoi(row[__LINK_TYPE__]);
        gwpi_tmp_2.wp.push_back(wpi_tmp); // Set Init WPs

        if(gwpi_tmp.size() == 0)
        {
            gwpi_tmp_2.nID = nID_tmp++;
            gwpi_tmp.push_back(gwpi_tmp_2);
            continue;
        }

        //        for(int i = 0; i < gwpi.size(); i++)
        if(gwpi_tmp.size() > 0)
        {
            int num_tmp = gwpi_tmp.size() - 1;
            if(gwpi_tmp[num_tmp].id == gwpi_tmp_2.id)
                gwpi_tmp[num_tmp].wp.push_back(wpi_tmp);
            else
            {
                gwpi_tmp_2.nID = nID_tmp++;
                gwpi_tmp.push_back(gwpi_tmp_2);
            }
        }
    }
    fin_tmp.close();

    for(int i = 0; i < gwpi_tmp.size(); i++)
        for(int j = 0; j < gwpi_tmp[i].wp.size(); j++)
            gwpi_tmp[i].wp[j].parent_node_nID = gwpi_tmp[i].nID;

//    std::vector<DoubGWPI> gwpi;
    gwpi.clear();
    for(int i = 0; i < gwpi_tmp.size(); i++)
    {
        double init_cost_tmp = 1e+38;
        gwpi.push_back({gwpi_tmp[i].id, gwpi_tmp[i].nID, __NULL__, __NULL__, {}, {}, {}, {}, __NULL__, gwpi_tmp[i].length, gwpi_tmp[i].max_speed, gwpi_tmp[i].link_type, init_cost_tmp, __NULL__, false, gwpi_tmp[i].wp});
    }
    for(int i = 0; i < gwpi_tmp.size(); i++)
    {
        for(int j = 0; j < gwpi_tmp.size(); j++)
        {
            /* Set Left/Right Link ID */
            if(gwpi_tmp[j].r_link_nID == gwpi_tmp[i].id)
                gwpi[j].r_link_nID = i;
            if(gwpi_tmp[j].l_link_nID == gwpi_tmp[i].id)
                gwpi[j].l_link_nID = i;

            /* Set From/To Node ID */
            if(gwpi_tmp[i].f_node_nID == gwpi_tmp[j].t_node_nID)
            {
                gwpi[i].f_node_nID.push_back(j);
                gwpi[j].t_node_nID.push_back(i);
            }
        }
    }
    for(int i = 0; i < gwpi.size(); i++)
    {
        if(gwpi[i].f_node_nID.size() == 0)
            gwpi[i].f_node_nID.push_back(__NULL__);
        if(gwpi[i].t_node_nID.size() == 0)
            gwpi[i].t_node_nID.push_back(__NULL__);
    }

    gli.clear();
    gli = GetLineInfoTreeMap(gwpi, utm_xy_offset_);

    /* gwpi 의 left, right line 의 nID를 가져옴. */
    for(int i = 0; i < gwpi.size(); i++)
    {
        for(int j = 0; j < gli.size(); j++)
        {
            if(i == gli[j].l_link_nID)
                gwpi[i].r_line_nID.push_back(j);

            if(i == gli[j].r_link_nID)
                gwpi[i].l_line_nID.push_back(j);
        }
    }
    for(int i = 0; i < gwpi.size(); i++)
    {
        if(gwpi[i].r_line_nID.size() == 0)
            gwpi[i].r_line_nID.push_back(__NULL__);
        if(gwpi[i].l_line_nID.size() == 0)
            gwpi[i].l_line_nID.push_back(__NULL__);
    }

    /* 차선 정보 입력 */
    for(int i = 0; i < gwpi.size(); i++)
    {
        int lane_no_tmp = 1;
        RecurCheckLaneNo(gwpi, i, lane_no_tmp);
        gwpi[i].lane_no = lane_no_tmp;
    }
//    return gwpi;
}


void WayPointsInterpolating(std::vector<WaypointsInfo> &global_waypoints, const double inter_dist_between_wps/* Unit: meter*/)
{
    for(int i = 0; i < global_waypoints.size()-1; i++)
    {
        double dist_x_tmp = global_waypoints[i].x - global_waypoints[i+1].x;
        double dist_y_tmp = global_waypoints[i].y - global_waypoints[i+1].y;
        if(std::hypot(dist_x_tmp, dist_y_tmp) > inter_dist_between_wps)
        {
//            std::cout << "i : " << i << std::endl;
            WaypointsInfo wp_tmp;
            double lat_half = (global_waypoints[i].latitude + global_waypoints[i+1].latitude) / 2;
            double long_half = (global_waypoints[i].longitude + global_waypoints[i+1].longitude) / 2;
            double x_half = (global_waypoints[i].x + global_waypoints[i+1].x) / 2;
            double y_half = (global_waypoints[i].y + global_waypoints[i+1].y) / 2;
            double angle_half = (global_waypoints[i].angle + global_waypoints[i+1].angle) / 2;
            double distance_half = (global_waypoints[i].distance + global_waypoints[i+1].distance) / 2;
            int node_nID_tmp = global_waypoints[i].parent_node_nID;
            wp_tmp = {lat_half, long_half, x_half, y_half, angle_half, distance_half, node_nID_tmp};
            global_waypoints.insert(global_waypoints.begin() + i + 1, wp_tmp);
            i--;
        }
    }

    // FIXME: 차선 변경 구간에서 Beyesian Polyfit 필요.
}

void RecurCheckLaneNo(const std::vector<DoubGWPI> gwpi_, const int id_, int &lane_no_)
{
    if(gwpi_[id_].l_link_nID != __NULL__)
    {
        int l_link_nID = gwpi_[id_].l_link_nID;
        RecurCheckLaneNo(gwpi_, l_link_nID, ++lane_no_);
    }
};

std::vector<std::string> csv_read_row(std::istream &in, char delimiter)
{
    std::stringstream ss;
    bool inquotes = false;
    std::vector<std::string> row;//relying on RVO
    while(in.good())
    {
        char c = in.get();
        if (!inquotes && c=='"') //beginquotechar
        {
            inquotes=true;
        }
        else if (inquotes && c=='"') //quotechar
        {
            if ( in.peek() == '"')//2 consecutive quotes resolve to 1
            {
                ss << (char)in.get();
            }
            else //endquotechar
            {
                inquotes=false;
            }
        }
        else if (!inquotes && c==delimiter) //end of field
        {
            row.push_back( ss.str() );
            ss.str("");
        }
        else if (!inquotes && (c=='\r' || c=='\n') )
        {
            if(in.peek()=='\n') { in.get(); }
            row.push_back( ss.str() );
            return row;
        }
        else
        {
            ss << c;
        }
    }
}

std::vector<std::string> csv_read_row(std::string &line, char delimiter)
{
    std::stringstream ss(line);
    return csv_read_row(ss, delimiter);
}

std::vector<DoubGLI> GetLineInfoTreeMap(const std::vector<DoubGWPI> gwpi, const std::pair<double, double> utm_xy_offset_)
{
    ros::NodeHandle nh_get_global_line_file;
    std::string file_dir;
    std::string param_name = "/linepoints_map/EntireLineFile";
    nh_get_global_line_file.getParam(param_name, file_dir);

    std::vector<StrGLI> str_gli_tmp;
    std::string csv_name = "LineMarkPoints(20210101).csv";
    std::ifstream fin_tmp(file_dir);
    std::string cell_tmp;
    std::string line_tmp;
    bool skip_flag = true;
    int nID_tmp = 0;
    while(fin_tmp.good())
    {
        std::vector<std::string> row = csv_read_row(fin_tmp, ',');

        // csv_read_row 다음에 나와야함.
        if(skip_flag == true)
        {
            skip_flag = false;
            continue;
        }
        std::pair<double, double> utm_xy_tmp;
        LatLonToUTMXY(std::stod(row[__LINE_LATITUDE__]), std::stod(row[__LINE_LONGITUDE__]), 0, utm_xy_tmp.first, utm_xy_tmp.second);

        WPI wpi_tmp;
        wpi_tmp.latitude = std::stod(row[__LINE_LATITUDE__]);
        wpi_tmp.longitude = std::stod(row[__LINE_LONGITUDE__]);
        wpi_tmp.x = utm_xy_tmp.first - utm_xy_offset_.first;
        wpi_tmp.y = utm_xy_tmp.second - utm_xy_offset_.second;
        wpi_tmp.angle = std::stod(row[__LINE_ANGLE__]);
        wpi_tmp.distance = std::stod(row[__LINE_DISTANCE__]);
        wpi_tmp.parent_node_nID = __NULL__;

        StrGLI gli_tmp_2;
        gli_tmp_2.id = row[__LINE_ID__];
//        gli_tmp_2.nID = nID_tmp++;
        gli_tmp_2.type = std::stod(row[__LINE_TYPE__]);
        gli_tmp_2.kind = std::stod(row[__LINE_KIND__]);
        gli_tmp_2.l_link_nID = row[__LINE_L_LINK_ID__];
        gli_tmp_2.r_link_nID = row[__LINE_R_LINK_ID__];
        gli_tmp_2.wp.push_back(wpi_tmp); // Set Init WPs

        if(str_gli_tmp.size() == 0)
        {
            gli_tmp_2.nID = nID_tmp++;
            str_gli_tmp.push_back(gli_tmp_2);
            continue;
        }

        if(str_gli_tmp.size() > 0)
        {
            int num_tmp = str_gli_tmp.size() - 1;
            if(str_gli_tmp[num_tmp].id == gli_tmp_2.id)
                str_gli_tmp[num_tmp].wp.push_back(wpi_tmp);
            else
            {
                gli_tmp_2.nID = nID_tmp++;
                str_gli_tmp.push_back(gli_tmp_2);
            }
        }
    }
    fin_tmp.close();

    for(int i = 0; i < str_gli_tmp.size(); i++)
        for(int j = 0; j < str_gli_tmp[i].wp.size(); j++)
            str_gli_tmp[i].wp[j].parent_node_nID = str_gli_tmp[i].nID;

    std::vector<DoubGLI> gli;
    for(int i = 0; i < str_gli_tmp.size(); i++)
    {
        gli.push_back({str_gli_tmp[i].id, str_gli_tmp[i].nID, str_gli_tmp[i].type, str_gli_tmp[i].kind, __NULL__, __NULL__, str_gli_tmp[i].wp});
    }

    for(int i = 0; i < gwpi.size(); i++)
    {
        for(int j = 0; j < str_gli_tmp.size(); j++)
        {
            std::string gli_r_link_id = str_gli_tmp[j].r_link_nID;
            std::string gli_l_link_id = str_gli_tmp[j].l_link_nID;
            if(gli_r_link_id == gwpi[i].id)
            {
                gli[j].r_link_nID = gwpi[i].nID;
            }

            if(gli_l_link_id == gwpi[i].id)
            {
                gli[j].l_link_nID = gwpi[i].nID;
            }
        }
    }

//    // FIXME:
//    for(int i = 0; i < str_gli_tmp.size(); i++)
//    {
//        for(int j = 0; j < str_gli_tmp.size(); j++)
//        {
//            /* Set Left/Right Link ID */
//            if(str_gli_tmp[j].r_link_nID == str_gli_tmp[i].id)
//                gli[j].r_link_nID = i;
//            if(str_gli_tmp[j].l_link_nID == str_gli_tmp[i].id)
//                gli[j].l_link_nID = i;

////            /* Set From/To Node ID */
////            if(gli_tmp[i].f_node_nID == gli_tmp[j].t_node_nID)
////            {
////                gli[i].f_node_nID.push_back(j);
////                gli[j].t_node_nID.push_back(i);
////            }
//        }
//    }

//    for(int i = 0; i < gli.size(); i++)
//    {
//        if(gli[i].f_node_nID.size() == 0)
//            gli[i].f_node_nID.push_back(__NULL__);
//        if(gli[i].t_node_nID.size() == 0)
//            gli[i].t_node_nID.push_back(__NULL__);
//    }

    return gli;
}

std::pair<double, double> CoordConvert_GlobalToVehicle(const std::pair<double, double> global_xy_, const std::pair<double, double> global_ego_xy_, const double orientation_rad_)
{
    double x_old = global_xy_.first - global_ego_xy_.first;
    double y_old = global_xy_.second - global_ego_xy_.second;
    double x_tmp = std::cos(-orientation_rad_) * x_old - std::sin(-orientation_rad_) * y_old;
    double y_tmp = std::sin(-orientation_rad_) * x_old + std::cos(-orientation_rad_) * y_old;
    return {x_tmp, y_tmp};
}
