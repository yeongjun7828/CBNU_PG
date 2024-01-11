//
// Created by YJ on 22. 10. 3.
//

#include "simple_path_ctrack/simple_path_ctrack.h"

using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "global_path");
    Simple_Path PP;
    ros::spin();
}