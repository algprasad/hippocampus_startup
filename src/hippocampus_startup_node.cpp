#include <iostream>
#include "ros/ros.h"
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Scalar.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

mavros_msgs::AttitudeTarget attitude_target_msg;
nav_msgs::Odometry odometry_msg;
static int seq = 0;

///New types of messages
geometry_msgs::PoseStamped cmd_att;
geometry_msgs::PoseWithCovarianceStamped mav_pose_with_covariance;
std_msgs::Float64 cmd_thr;
mavros_msgs::State current_state;


//transform broadcasters




void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void initializeOdometry() {
    odometry_msg.header.stamp = ros::Time::now();
    odometry_msg.header.seq = seq++;

    odometry_msg.pose.pose.position.x = 0;
    odometry_msg.pose.pose.position.y = 0;
    odometry_msg.pose.pose.position.z = 0;

    odometry_msg.pose.pose.orientation.x = 0;
    odometry_msg.pose.pose.orientation.y = 0;
    odometry_msg.pose.pose.orientation.z = 0;
    odometry_msg.pose.pose.orientation.w = 1;

    odometry_msg.twist.twist.linear.x = 0;
    odometry_msg.twist.twist.linear.y = 0;
    odometry_msg.twist.twist.linear.z = 0;

    odometry_msg.twist.twist.angular.x = 0;
    odometry_msg.twist.twist.angular.y = 0;
    odometry_msg.twist.twist.angular.z = 0;

}

void mavrosGlobalPositionCallback(const geometry_msgs::PoseStampedPtr& msg){
    odometry_msg.pose.pose.position.x = msg->pose.position.x;
    odometry_msg.pose.pose.position.y = msg->pose.position.y;
    odometry_msg.pose.pose.position.z = msg->pose.position.z;

    odometry_msg.pose.pose.orientation.x = msg->pose.orientation.x;
    odometry_msg.pose.pose.orientation.y = msg->pose.orientation.y;
    odometry_msg.pose.pose.orientation.z = msg->pose.orientation.z;
    odometry_msg.pose.pose.orientation.w = msg->pose.orientation.w;

    //get the pose with covariance message
    mav_pose_with_covariance.pose.pose.position.x =  msg->pose.position.x;
    mav_pose_with_covariance.pose.pose.position.y =  msg->pose.position.y;
    mav_pose_with_covariance.pose.pose.position.z =  msg->pose.position.z;

    mav_pose_with_covariance.pose.pose.orientation.x = msg->pose.orientation.x;
    mav_pose_with_covariance.pose.pose.orientation.y = msg->pose.orientation.y;
    mav_pose_with_covariance.pose.pose.orientation.z = msg->pose.orientation.z;
    mav_pose_with_covariance.pose.pose.orientation.w = msg->pose.orientation.w;

    mav_pose_with_covariance.header.stamp = ros::Time::now();

    //TODO: publish the tf for base_link in terms of world frame
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z) );
    tf::Quaternion q;
    //q.setRPY(0, 0, msg->theta);
    q.setX(msg->pose.orientation.x);
    q.setY(msg->pose.orientation.y);
    q.setZ(msg->pose.orientation.z);
    q.setW(msg->pose.orientation.w);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));

}


void initializeAttitudeTarget(){
    attitude_target_msg.header.stamp = ros::Time::now();
    attitude_target_msg.header.seq = seq++;

    attitude_target_msg.orientation.x = 0;
    attitude_target_msg.orientation.y = 0; //-0.707106781187;
    attitude_target_msg.orientation.z = 0;
    attitude_target_msg.orientation.w = 1; // 0.707106781187;

    attitude_target_msg.thrust = 0.5; //0.5; // + seq*0.01;

}

void hippocampusAttitudeTargetCallback(mavros_msgs::AttitudeTargetConstPtr msg){
    attitude_target_msg.header.stamp = ros::Time::now();
    attitude_target_msg.header.seq = seq++;

    attitude_target_msg.orientation = msg->orientation;
    attitude_target_msg.thrust = msg->thrust;

}





int main(int argc, char **argv){
    ros::init(argc, argv, "rostopic_modulator");
    ros::NodeHandle nh;

    // Publisher and Subsriber stuff
    ros::Publisher pub_mavros_setpoint_attitude = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1000);

    ros::Subscriber sub_mavros_global_position = nh.subscribe("/mavros/local_position/pose", 1000, mavrosGlobalPositionCallback);

    //Subcriber to the attitude target message from hippocampus_pid_control
    ros::Subscriber sub_attitude_target_hippocampus = nh.subscribe("/hippocampus/setpoint_raw/attitude", 1000, hippocampusAttitudeTargetCallback);



    //mavros commanding subscribers and publishers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    initializeOdometry();
    initializeAttitudeTarget();

    ros::Rate rate(50);

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




        pub_mavros_setpoint_attitude.publish(attitude_target_msg);




        //pub_att.publish(cmd_att);
        //pub_thr.publish(cmd_thr);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;


}


