#include "ros/ros.h"

#include <tf/transform_listener.h>
// #include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Twist.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <sstream>

#include <termios.h>
#include <stdio.h>
#include <iostream>

/* crazyflie_driver */
#include "crazyflie_driver/AddCrazyflie.h"
#include "crazyflie_driver/LogBlock.h"
#include "crazyflie_driver/GenericLogData.h"
#include "crazyflie_driver/UpdateParams.h"
#include "crazyflie_driver/UploadTrajectory.h"
#include "crazyflie_driver/NotifySetpointsStop.h"
#undef major
#undef minor
#include "crazyflie_driver/Hover.h"
#include "crazyflie_driver/Takeoff.h"
#include "crazyflie_driver/Land.h"
#include "crazyflie_driver/GoTo.h"
#include "crazyflie_driver/StartTrajectory.h"
#include "crazyflie_driver/SetGroupMask.h"
#include "crazyflie_driver/FullState.h"
#include "crazyflie_driver/Position.h"
#include "crazyflie_driver/VelocityWorld.h"

/* Includes form iLQR main.cpp */
#include <unsupported/Eigen/AdolcForward>
#include <Eigen/Dense>
#include <adolc/adolc.h>
#include <adolc/adouble.h>
#include <vector>
#include <time.h>
#include <math.h>
#include <fstream>
#include <thread>
#include <future>
#include <algorithm>
#include <iterator>
#include "cost.h"
#include "dynamics.h"
#include "iLQR.h"
#include "utils.h"
#include <unistd.h>


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

    // std::cout << "timeStamp: " << timeStamp << std::endl;

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
    std::printf("##### CALLBACK\n");

    std::printf("timeStamp: %lu\n", timeStamp);
    std::printf("deltaT: %f\n", deltaT);

    std::printf("X: %f,\t Y: %f,\t Z: %f\t\n ROLL: %f,\t PITCH: %f,\t YAW %f\n",
                CFState.x, CFState.y, CFState.z, CFState.roll, CFState.pitch, CFState.yaw);

    std::printf("X_d: %f\t, Y_d: %f\t, Z_d: %f\t\nROLL_d: %f\t, PITCH_d: %f\t, YAW_d %f\n",
                CFState.x_d, CFState.y_d, CFState.z_d, CFState.roll_d, CFState.pitch_d, CFState.yaw_d);
}

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  newt.c_cc[VMIN] = 0; newt.c_cc[VTIME] = 0;
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "iLQR_Node");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("tf", 1000, callback);

    ros::Publisher pub = n.advertise<crazyflie_driver::Position>("cf1/cmd_position",1);

    ros::ServiceClient takeoffClient = n.serviceClient<crazyflie_driver::Takeoff>("/takeoff");
    ros::ServiceClient landClient = n.serviceClient<crazyflie_driver::Land>("/land");
    ros::ServiceClient GoToClient = n.serviceClient<crazyflie_driver::GoTo>("/cf1/go_to");

    crazyflie_driver::Position position_cmd;

    /* Send takeoff command through /Takeoff service */
    crazyflie_driver::Takeoff srvTakeoff;
    srvTakeoff.request.groupMask = 0;
    srvTakeoff.request.height = 0.5;
    srvTakeoff.request.duration = ros::Duration(6.0);
    takeoffClient.call(srvTakeoff);

    ros::Duration(5.0).sleep();

    crazyflie_driver::GoTo srvGoTo;
    srvGoTo.request.groupMask = 0;

    // srvGoTo.request.goal.x = 0.0;
    // srvGoTo.request.goal.y = 0.0;
    // srvGoTo.request.goal.z = 2.0;
    // srvGoTo.request.yaw = 0.0;
    // srvGoTo.request.duration = ros::Duration(3.0);
    // GoToClient.call(srvGoTo);
    //
    // ros::Duration(1.0).sleep();
    //
    // srvGoTo.request.goal.x = 0.0;
    // srvGoTo.request.goal.y = 1.0;
    // srvGoTo.request.goal.z = 2.0;
    // srvGoTo.request.yaw = 0.0;
    // srvGoTo.request.duration = ros::Duration(3.0);
    // GoToClient.call(srvGoTo);
    //
    // ros::Duration(1.0).sleep();
    //
    // srvGoTo.request.goal.x = 1.0;
    // srvGoTo.request.goal.y = 1.0;
    // srvGoTo.request.goal.z = 2.0;
    // srvGoTo.request.yaw = 0.0;
    // srvGoTo.request.duration = ros::Duration(3.0);
    // GoToClient.call(srvGoTo);
    //
    // ros::Duration(1.0).sleep();
    //
    // srvGoTo.request.goal.x = 1.0;
    // srvGoTo.request.goal.y = 0.0;
    // srvGoTo.request.goal.z = 2.0;
    // srvGoTo.request.yaw = 0.0;
    // srvGoTo.request.duration = ros::Duration(3.0);
    // GoToClient.call(srvGoTo);
    //
    // ros::Duration(1.0).sleep();
    //
    // srvGoTo.request.goal.x = 0.0;
    // srvGoTo.request.goal.y = 0.0;
    // srvGoTo.request.goal.z = 2.0;
    // srvGoTo.request.yaw = 0.0;
    // srvGoTo.request.duration = ros::Duration(3.0);
    // GoToClient.call(srvGoTo);
    //
    // ros::Duration(1.0).sleep();


    constexpr size_t horizon=5;
    constexpr int total_steps=300;
////	int horizon=1000;
    unsigned int tag1(1),tag2(2),tag3(3),tag4(4),tag5(5),tag6(6)
    ,tag7(7),tag8(8),tag9(9);

    double time_step=0.1;

//

// /* This part is single drone simulation*/
//
//     cost<12,4> running_cost_dr(Drone_Cost::running_cost,tag1);
//     cost<12,4> terminal_cost_dr(Drone_Cost::terminal_cost,tag2);
//     dynamics<12,4> dynamics_dr(Drone_Dynamics::dynamics,tag3,time_step);
//
//     iLQR<12,4,horizon> drone_solver(running_cost_dr,terminal_cost_dr,dynamics_dr);
//     Eigen::Matrix<double,4,1> u_drone=Eigen::Matrix<double,4,1>::Ones();
//     Eigen::Matrix<double,12,1> X0_drone=Eigen::Matrix<double,12,1>::Zero();
// ////	X0_drone<<0,-50,10, M_PI/20,0,0, 0,0,0, 0,0,0;
//     Eigen::Matrix<double,12,1> X_goal_drone;
//
//     u_drone=u_drone*sqrt(Drone_Dynamics::mass*Drone_Dynamics::g/(4*Drone_Dynamics::C_T));
// //
//     iLQR<12,4,horizon>::input_trajectory u_init_drone;
//     std::fill(std::begin(u_init_drone), std::end(u_init_drone), u_drone);
//     iLQR<12,4,horizon>::state_input_trajectory soln_drone;
//     iLQR<12,4,total_steps>::state_input_trajectory soln_drone_rhc;
//
//     std::cout << "DEBUG 1" << std::endl;
//
//     clock_t t_start, t_end;
//
// //
//     double seconds;
//     drone_solver.set_MPC(u_init_drone);
//     int execution_steps=1;
//     soln_drone_rhc.first[0]=X0_drone;
//     // std::string state_path="/Users/talhakavuncu/Desktop/research/cpp_code/states.txt";
//     // std::string input_path="/Users/talhakavuncu/Desktop/research/cpp_code/inputs.txt";
//     t_start = clock();
//     // for(int i=0;i<total_steps;++i)
//
//
//     X_goal_drone<<1.0,1.0,1.5, 0,0,0, 0,0,0, 0,0,0;
//
//
//     while (ros::ok())
//     {
//         // if (i>total_steps/3)
//         //     X_goal_drone<<0,0,0, 0,0,0, 0,0,0, 0,0,0;
//         // std::cout<<"iteration "<<i<<std::endl;
//
//         ros::spinOnce();
// ////
//         X0_drone << CFState.x, CFState.y, CFState.z,
//                     CFState.roll, CFState.pitch,  CFState.yaw,
//                     CFState.x_d, CFState.y_d, CFState.z_d,
//                     CFState.roll_d, CFState.pitch_d, CFState.yaw_d;
//
//         soln_drone=drone_solver.run_MPC(X0_drone, X_goal_drone, 100, execution_steps);
//
//         // ros::Duration(0.5).sleep();
// //
// //////		unsigned int microsecond = 1000000;
// //////		usleep(2 * microsecond);
// //		write_file<12,4,horizon>(state_path,input_path, soln_drone);
//         // soln_drone_rhc.first[i+1]=soln_drone.first[1];
//         // soln_drone_rhc.second[i]=soln_drone.second[0];
//
//         std::printf("\n\n");
//         std::cout << X0_drone << std::endl;
//         std::printf("X: %f,\t Y: %f,\t Z: %f\n", soln_drone.first[39][0], soln_drone.first[39][1], soln_drone.first[39][2]);
//
//         srvGoTo.request.goal.x = soln_drone.first[39][0];
//         srvGoTo.request.goal.y = soln_drone.first[39][1];
//         srvGoTo.request.goal.z = soln_drone.first[39][2];
//         srvGoTo.request.yaw = 0.0;
//         srvGoTo.request.duration = ros::Duration(5.0);
//         GoToClient.call(srvGoTo);
//
//         int c = getch();   // call your non-blocking input function
//         if (c == '\n') {
//             std::cout << "LANDING" << std::endl;
//             break;
//         }
//
//         // ros::spinOnce();
//
//     }
//     t_end = clock();
//
//
// ////
// ////
// //////	soln_drone=drone_solver.solve_open_loop(X0_drone, X0_drone, 200, u_init_drone, horizon);
// //////	write_file<12,4,horizon>(state_path,input_path, soln_drone);
//     // write_file<12,4,total_steps>(state_path,input_path, soln_drone_rhc);
// //
//     std::cout<<"finished"<<std::endl;
//     seconds = 1/((double)(t_end - t_start) / CLOCKS_PER_SEC/total_steps);
//     printf("Time: %f s\n", seconds);
//
//
// /* End of single drone simulation*/





cost<3,3> integrator_running_cost(Single_Integrator_Cost::running_cost,tag7);
cost<3,3> integrator_terminal_cost(Single_Integrator_Cost::terminal_cost,tag8);
dynamics<3,3> integrator_dynamics(Single_Integrator_3D::dynamics,tag9,time_step);
iLQR<3,3,horizon>::input_trajectory u_init_integrator;
iLQR<3,3,horizon> integrator_solver(integrator_running_cost,integrator_terminal_cost,integrator_dynamics);
Eigen::Matrix<double,3,1> u_integrator=Eigen::Matrix<double,3,1>::Zero();
Eigen::Matrix<double,3,1> x0_integrator=Eigen::Matrix<double,3,1>::Zero();
Eigen::Matrix<double,3,1> x_goal_integrator;
iLQR<3,3,horizon>::state_input_trajectory soln_integrator;
iLQR<3,3,total_steps>::state_input_trajectory soln_integrator_rhc;

//	u_drone2=u_drone2*sqrt(Drone_Dynamics::mass*Drone_Dynamics::g/(4*Drone_Dynamics::C_T));
//	std::fill(std::begin(u_init_drone2), std::end(u_init_drone2), u_drone2);
//	iLQR<12*2,4*2,horizon>::state_input_trajectory soln_drone2;
//	iLQR<12*2,4*2,total_steps>::state_input_trajectory soln_drone2_rhc;





	clock_t t_start, t_end;
//
////
	double seconds;
	integrator_solver.set_MPC(u_init_integrator);
	int execution_steps=1;
	soln_integrator_rhc.first[0]=x0_integrator;
	// std::string state_path="/Users/talhakavuncu/Desktop/research/cpp_code/states.txt";
	// std::string input_path="/Users/talhakavuncu/Desktop/research/cpp_code/inputs.txt";
	t_start = clock();
	// for(int i=0;i<total_steps;++i)


    x_goal_integrator<<1.0,1.0,1.0;


    while (ros::ok())
	{
//		if (i>total_steps/3)
//			x_goal_integrator<<0,0,0;
		// std::cout<<"iteration "<<i<<std::endl;

        ros::spinOnce();
//////

        x0_integrator << CFState.x, CFState.y, CFState.z;

		soln_integrator=integrator_solver.run_MPC(x0_integrator,
				x_goal_integrator, 50, execution_steps);
////
////////		unsigned int microsecond = 1000000;
////////		usleep(2 * microsecond);
////		write_file<12,4,horizon>(state_path,input_path, soln_drone);
		// soln_integrator_rhc.first[i+1]=soln_integrator.first[1];
		// soln_integrator_rhc.second[i]=soln_integrator.second[0];

        std::printf("\n\n");
        std::cout << x0_integrator << std::endl;
        std::printf("X: %f,\t Y: %f,\t Z: %f\n", soln_integrator.first[4][0], soln_integrator.first[4][4], soln_integrator.first[4][2]);

        srvGoTo.request.goal.x = soln_integrator.first[4][0];
        srvGoTo.request.goal.y = soln_integrator.first[4][1];
        srvGoTo.request.goal.z = soln_integrator.first[4][2];
        srvGoTo.request.yaw = 0.0;
        srvGoTo.request.duration = ros::Duration(3.0);
        GoToClient.call(srvGoTo);

        int c = getch();   // call your non-blocking input function
        if (c == '\n') {
            std::cout << "LANDING" << std::endl;
            break;
        }


	}
	t_end = clock();
////
//////
////////	soln_drone=drone_solver.solve_open_loop(X0_drone, X0_drone, 200, u_init_drone, horizon);
////////	write_file<12,4,horizon>(state_path,input_path, soln_drone);
	// write_file<3,3,total_steps>(state_path,input_path, soln_integrator_rhc);
////
	std::cout<<"finished"<<std::endl;
	seconds = 1/((double)(t_end - t_start) / CLOCKS_PER_SEC/total_steps);
	printf("Time: %f s\n", seconds);






    // // /* Control loop */
    // while (ros::ok()) {
    //
    //     int c = getch();   // call your non-blocking input function
    //     if (c == '\n') {
    //         std::cout << "LANDING" << std::endl;
    //         break;
    //     }
    //
    //     ros::spinOnce();
    //
    // 	// position_cmd.x = 0;
    // 	// position_cmd.y = 0;
    // 	// position_cmd.z = 2.0;
    // 	// position_cmd.yaw = 0.0;
    // 	// pub.publish(position_cmd);
    //     //
    //     // ros::spinOnce();
    //
    //
    // }

    crazyflie_driver::Land srvLand;
    srvLand.request.duration = ros::Duration(4.0);
    landClient.call(srvLand);

    ros::spin();

    return 0;
}
