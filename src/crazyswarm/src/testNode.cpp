#include "ros/ros.h"

#include <tf/transform_listener.h>
// #include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Twist.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <sstream>

#include <iostream>

struct stateVector {
    float x, y, z;
    float roll, pitch, yaw;
    float x_d, y_d, z_d;
    float roll_d, pitch_d, yaw_d;
};

struct pose {
    float x, y, z;
    float roll, pitch, yaw;
    unsigned long timeStamp;
};

static stateVector CFState;

static pose currentPose;
static pose prevPose;
bool first = true;

void callback(const tf2_msgs::TFMessage::ConstPtr& msg) {

    // ROS_INFO("##### /tf MESSAGE RECEIVED #####");

    // std::cout << msg->transforms[0].transform.translation.x << ", "
    // << msg->transforms[0].transform.translation.y << ", "
    // << msg->transforms[0].transform.translation.z << std::endl;

    float x = msg->transforms[0].transform.translation.x;
    float y = msg->transforms[0].transform.translation.y;
    float z = msg->transforms[0].transform.translation.z;

    float qx = msg->transforms[0].transform.rotation.x;
    float qy = msg->transforms[0].transform.rotation.y;
    float qz = msg->transforms[0].transform.rotation.z;
    float qw = msg->transforms[0].transform.rotation.w;
    tf2::Quaternion q(qx, qy, qz, qw);

    double roll, pitch, yaw;
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    // std::cout << roll << ", " << pitch << ", " << yaw << std::endl;

    int secs = msg->transforms[0].header.stamp.sec;
    int nsecs = msg->transforms[0].header.stamp.nsec;

    // std::cout << msg->transforms[0].header.stamp.sec << std::endl;
    // std::cout << secs << std::endl;

    // double timeStamp = secs + (nsecs * 0.000000001);
    unsigned long timeStamp = (secs * 1e9) + nsecs;

    std::cout << "timeStamp: " << timeStamp << std::endl;

    double deltaT = 0.0;

    if (first == true) {

        currentPose.x = x;
        currentPose.y = y;
        currentPose.z = z;
        currentPose.roll = roll;
        currentPose.pitch = pitch;
        currentPose.yaw = yaw;
        currentPose.timeStamp = timeStamp;

        prevPose.x = x;
        prevPose.y = y;
        prevPose.z = z;
        prevPose.roll = roll;
        prevPose.pitch = pitch;
        prevPose.yaw = yaw;
        prevPose.timeStamp = timeStamp;

        first = false;

    } else {

        currentPose.x = x;
        currentPose.y = y;
        currentPose.z = z;
        currentPose.roll = roll;
        currentPose.pitch = pitch;
        currentPose.yaw = yaw;
        currentPose.timeStamp = timeStamp;

        deltaT = (currentPose.timeStamp - prevPose.timeStamp) * 1e-9;

        CFState.x = x;
        CFState.y = y;
        CFState.z = z;
        CFState.roll = roll;
        CFState.pitch = pitch;
        CFState.yaw = yaw;
        CFState.x_d = (currentPose.x - prevPose.x) / deltaT;
        CFState.y_d = (currentPose.y - prevPose.y) / deltaT;
        CFState.z_d = (currentPose.z - prevPose.z) / deltaT;
        CFState.roll_d = (currentPose.roll - prevPose.roll) / deltaT;
        CFState.pitch_d = (currentPose.pitch - prevPose.pitch) / deltaT;
        CFState.yaw_d = (currentPose.yaw - prevPose.yaw) / deltaT;

        prevPose = currentPose;

    }

    std::printf("\n\n");

    std::printf("timeStamp: %lu\n", timeStamp);
    std::printf("deltaT: %f\n", deltaT);

    std::printf("X: %f, Y: %f, Z: %f, ROLL: %f, PITCH: %f, YAW %f\n",
                CFState.x, CFState.y, CFState.z, CFState.roll, CFState.pitch, CFState.yaw);

    std::printf("X_d: %f\t, Y_d: %f\t, Z_d: %f\t\nROLL_d: %f\t, PITCH_d: %f\t, YAW_d %f\n",
                CFState.x_d, CFState.y_d, CFState.z_d, CFState.roll_d, CFState.pitch_d, CFState.yaw_d);
}



int main(int argc, char **argv) {

    ros::init(argc, argv, "testNode");

    ros::NodeHandle n;

    tf::TransformListener listener;

    ros::Subscriber sub = n.subscribe("tf", 1000, callback);



    ros::spin();

    return 0;
}
