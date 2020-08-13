#include <iostream>

int main(){
    ros::init(argc, argv, "position_controller");
    ros::NodeHandle nh;

    //subscribe
    ros::Subscriber sub_mavros_global_position = nh.subscribe("/mavros/local_position/pose", 1000, mavrosGlobalPositionCallback);



}