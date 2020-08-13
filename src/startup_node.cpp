#include <iostream>
#include "ros/ros.h"
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>


//All global variables
geometry_msgs::PoseWithCovarianceStamped mav_pose_with_covariance;
mavros_msgs::State current_state;
mavros_msgs::AttitudeTarget attitude_target_msg;
int seq= 0;

void mavrosGlobalPositionCallback(const geometry_msgs::PoseStampedConstPtr& msg){

    //get the pose with covariance message
    mav_pose_with_covariance.pose.pose.position.x =  msg->pose.position.x;
    mav_pose_with_covariance.pose.pose.position.y =  msg->pose.position.y;
    mav_pose_with_covariance.pose.pose.position.z =  msg->pose.position.z;

    mav_pose_with_covariance.pose.pose.orientation.x = msg->pose.orientation.x;
    mav_pose_with_covariance.pose.pose.orientation.y = msg->pose.orientation.y;
    mav_pose_with_covariance.pose.pose.orientation.z = msg->pose.orientation.z;
    mav_pose_with_covariance.pose.pose.orientation.w = msg->pose.orientation.w;

    mav_pose_with_covariance.header.stamp = ros::Time::now();

    //publish the tf for base_link in terms of world frame
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z) );
    tf::Quaternion q;

    q.setX(msg->pose.orientation.x);
    q.setY(msg->pose.orientation.y);
    q.setZ(msg->pose.orientation.z);
    q.setW(msg->pose.orientation.w);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));

}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void initializeAttitudeTarget(){
    attitude_target_msg.header.stamp = ros::Time::now();
    attitude_target_msg.header.frame_id = "world";

    attitude_target_msg.orientation.x = 0;
    attitude_target_msg.orientation.y = 0;
    attitude_target_msg.orientation.z = 0;
    attitude_target_msg.orientation.w = 1;

     //TODO: currently ignoring yaw rates too just for the initial test. CHANGE THIS IMMEDIATELY AFTER THE TEST.
    attitude_target_msg.type_mask= 7; //attitude_target_msg.IGNORE_PITCH_RATE + attitude_target_msg.IGNORE_ROLL_RATE + attitude_target_msg.IGNORE_YAW_RATE;
    attitude_target_msg.thrust = 0.5 + seq*0.01;
    attitude_target_msg.body_rate.x = 0;
    attitude_target_msg.body_rate.y = 0;
    attitude_target_msg.body_rate.z = 0;

}

void setAttitudeTarget(){
    attitude_target_msg.header.stamp = ros::Time::now();
    attitude_target_msg.header.seq = seq++;
    attitude_target_msg.header.frame_id =  "world";

    attitude_target_msg.orientation.x = 0;
    attitude_target_msg.orientation.y = 0;
    attitude_target_msg.orientation.z = 0;
    attitude_target_msg.orientation.w = 1;

    attitude_target_msg.thrust = 0.5 + seq*0.0001;
    attitude_target_msg.body_rate.x = 0;
    attitude_target_msg.body_rate.y = 0;
    attitude_target_msg.body_rate.z = 0;



    //TODO: currently ignoring yaw rates too just for the initial test. CHANGE THIS IMMEDIATELY AFTER THE TEST.
    attitude_target_msg.type_mask= 7; //attitude_target_msg.IGNORE_PITCH_RATE + attitude_target_msg.IGNORE_ROLL_RATE + attitude_target_msg.IGNORE_YAW_RATE;

}


int main(int argc, char **argv){
    ros::init(argc, argv, "startup_node");
    ros::NodeHandle nh;

    // Publisher and Subsriber stuff
    ros::Publisher pub_mavros_setpoint_attitude = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/target_attitude", 1000);

    ros::Subscriber sub_mavros_global_position = nh.subscribe("/mavros/local_position/pose", 1000, mavrosGlobalPositionCallback);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);


    //mavros commanding subscribers and publishers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, state_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");



    initializeAttitudeTarget();

    ros::Rate rate(20);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }



    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();



    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        //checkign if armed
        if(current_state.armed) ROS_INFO("ARMED MOFOS");



        setAttitudeTarget();
        pub_mavros_setpoint_attitude.publish(attitude_target_msg);

        //std::cout<<attitude_target_msg.thrust<<std::endl;



        ros::spinOnce();
        rate.sleep();
    }

    return 0;


}

