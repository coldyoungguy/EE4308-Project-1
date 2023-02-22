#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <errno.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include "common.hpp"
#include <std_msgs/Float32.h>
#include <fstream>

bool target_changed = false;
Position target;
void cbTarget(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    target.x = msg->point.x;
    target.y = msg->point.y;
}

Position pos_rbt(0, 0);
double ang_rbt = 10; // set to 10, because ang_rbt is between -pi and pi, and integer for correct comparison while waiting for motion to load
void cbPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    auto &p = msg->pose.position;
    pos_rbt.x = p.x;
    pos_rbt.y = p.y;

    // euler yaw (ang_rbt) from quaternion <-- stolen from wikipedia
    auto &q = msg->pose.orientation; // reference is always faster than copying. but changing it means changing the referenced object.
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    ang_rbt = atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_move");
    ros::NodeHandle nh;

    // Get ROS Parameters
    bool enable_move, record;
    if (!nh.param("enable_move", enable_move, true))
        ROS_WARN(" TMOVE : Param enable_move not found, set to true");
    bool verbose;
    if (!nh.param("verbose_move", verbose, false))
        ROS_WARN(" TMOVE : Param verbose_move not found, set to false");
    double Kp_lin;
    if (!nh.param("Kp_lin", Kp_lin, 1.0))
        ROS_WARN(" TMOVE : Param Kp_lin not found, set to 1.0");
    double Ki_lin;
    if (!nh.param("Ki_lin", Ki_lin, 0.0))
        ROS_WARN(" TMOVE : Param Ki_lin not found, set to 0");
    double Kd_lin;
    if (!nh.param("Kd_lin", Kd_lin, 0.0))
        ROS_WARN(" TMOVE : Param Kd_lin not found, set to 0");
    double max_lin_vel;
    if (!nh.param("max_lin_vel", max_lin_vel, 0.22))
        ROS_WARN(" TMOVE : Param max_lin_vel not found, set to 0.22");
    double max_lin_acc;
    if (!nh.param("max_lin_acc", max_lin_acc, 1.0))
        ROS_WARN(" TMOVE : Param max_lin_acc not found, set to 1");
    double Kp_ang;
    if (!nh.param("Kp_ang", Kp_ang, 1.0))
        ROS_WARN(" TMOVE : Param Kp_ang not found, set to 1.0");
    double Ki_ang;
    if (!nh.param("Ki_ang", Ki_ang, 0.0))
        ROS_WARN(" TMOVE : Param Ki_ang not found, set to 0");
    double Kd_ang;
    if (!nh.param("Kd_ang", Kd_ang, 0.0))
        ROS_WARN(" TMOVE : Param Kd_ang not found, set to 0");
    double max_ang_vel;
    if (!nh.param("max_ang_vel", max_ang_vel, 2.84))
        ROS_WARN(" TMOVE : Param max_ang_vel not found, set to 2.84");
    double max_ang_acc;
    if (!nh.param("max_ang_acc", max_ang_acc, 4.0))
        ROS_WARN(" TMOVE : Param max_ang_acc not found, set to 4");
    double move_iter_rate;
    if (!nh.param("move_iter_rate", move_iter_rate, 25.0))
        ROS_WARN(" TMOVE : Param move_iter_rate not found, set to 25");
    double turn_throttle_scale;
    if (!nh.param("turn_throttle_scale", turn_throttle_scale, 1.0))
        ROS_WARN(" TMOVE : Param turn_throttle_scale not found, set to 1");
    if (!nh.param("data_record", record, false))
        ROS_WARN(" TMAIN : Param data_record not found, set to false");
    std::string coupling_type;
    if (!nh.getParam("coupling_type", coupling_type))
        ROS_WARN(" TMAIN : Param coupling_type not found, set to exp");
        coupling_type = "exp";
    std::string PATH_record;
    if (!nh.getParam("PATH_record", PATH_record)) {
        ROS_WARN(" TMAIN : Param PATH_record not found, set data_record to false");
        record = false;
    }
    if (PATH_record == "PATH") {
        ROS_WARN(" TMAIN : Param PATH_record not set, set data_record to false");
        record = false;
    }

    // For file recording
    std::ofstream data_file;
    if (record) {
        data_file.open(PATH_record);
    }

    // Subscribers
    ros::Subscriber sub_target = nh.subscribe("target", 1, &cbTarget);
    ros::Subscriber sub_pose = nh.subscribe("pose", 1, &cbPose);

    // Publishers
    ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
    ros::Publisher error_pos_pub_ = nh.advertise<std_msgs::Float32>("/error_pos", 1); 
    ros::Publisher error_ang_pub_ = nh.advertise<std_msgs::Float32>("/error_ang", 1);

    // prepare published messages
    geometry_msgs::Twist msg_cmd; // all properties are initialised to zero.

    // Setup rate
    ros::Rate rate(move_iter_rate); // same as publishing rate of pose topic

    // wait for other nodes to load
    ROS_INFO(" TMOVE : Waiting for topics");
    while (ros::ok() && nh.param("run", true) && ang_rbt == 10) // not dependent on main.cpp, but on motion.cpp
    {
        rate.sleep();
        ros::spinOnce(); //update the topics
    }

    // Setup variables
    double cmd_lin_vel = 0, cmd_ang_vel = 0;
    double dt;
    double prev_time = ros::Time::now().toSec();

    ////////////////// DECLARE VARIABLES HERE //////////////////
    std_msgs::Float32 lin_error;
    std_msgs::Float32 ang_error;
    double error_pos, error_ang;
    double error_pos_prev = error_pos, error_ang_prev = error_ang;
    double I_pos, I_ang, D_pos, D_ang;
    double cmd_lin_vel_prev = cmd_lin_vel, cmd_ang_vel_prev = cmd_ang_vel;
    double cmd_lin_acc, cmd_ang_acc;

    ROS_INFO(" TMOVE : ===== BEGIN =====");

    // main loop
    if (enable_move)
    {
        while (ros::ok() && nh.param("run", true))
        {
            // update all topics
            ros::spinOnce();

            dt = ros::Time::now().toSec() - prev_time;
            if (dt == 0) // ros doesn't tick the time fast enough
                continue;
            prev_time += dt;

            ////////////////// MOTION CONTROLLER HERE //////////////////

            error_ang = limit_angle(heading(pos_rbt, target) - ang_rbt);
            if ((!sign(error_ang) && abs(error_ang) < M_PI_2)) {
                error_pos = -1 * dist_euc(pos_rbt, target);
                error_ang += M_PI;
            } else if (sign(error_ang) && abs(error_ang) > M_PI_2) {
                error_pos = -1 * dist_euc(pos_rbt, target);
                error_ang -= M_PI;
            } else {error_pos = dist_euc(pos_rbt, target);}
            

            I_pos += dt * error_pos;
            I_ang += dt * error_ang;
            D_pos = (error_pos - error_pos_prev) / dt;
            D_ang = (error_ang - error_ang_prev) / dt;

            cmd_lin_vel = Kp_lin * error_pos + Ki_lin * I_pos + Kd_lin * D_pos;
            cmd_ang_vel = Kp_ang * error_ang + Ki_ang * I_ang + Kd_ang * D_ang;            

            cmd_lin_acc = sat((cmd_lin_vel - cmd_lin_vel_prev) / dt, max_lin_acc);
            cmd_ang_acc = sat((cmd_ang_vel - cmd_ang_vel_prev) / dt, max_ang_acc);
            cmd_lin_vel = sat(cmd_lin_vel_prev + cmd_lin_acc * dt, max_lin_vel);
            cmd_ang_vel = sat(cmd_ang_vel_prev + cmd_ang_acc * dt, max_ang_vel);

            // curve for lin vel smoothing
            if (coupling_type == "exp") {
                // higher number for turn_throttle_scale make graph steeper
                // ROS_INFO("[Curve smoothing] Using exponential curve for lin vel smoothing");
                cmd_lin_vel = cmd_lin_vel * exp(M_PI- turn_throttle_scale * error_ang) / (1 + exp(M_PI- turn_throttle_scale * error_ang)) * 1.03;
            } else if (coupling_type == "cos") {
                // higher even number for turn_throttle_scale make graph steeper
                // ROS_INFO("[Curve smoothing] Using cosine curve for lin vel smoothing");
                cmd_lin_vel = cmd_lin_vel * pow(cos(error_ang), turn_throttle_scale);
            }

            // save current timestep for future calculations
            error_pos_prev = error_pos;
            error_ang_prev = error_ang;
            cmd_lin_vel_prev = cmd_lin_vel;
            cmd_ang_vel_prev = cmd_ang_vel;
            
            // publish speeds
            msg_cmd.linear.x = cmd_lin_vel;
            msg_cmd.angular.z = cmd_ang_vel;
            pub_cmd.publish(msg_cmd);

            // publish errors for rqt_plot
            lin_error.data = error_pos;
            error_pos_pub_.publish(lin_error);
            ang_error.data = error_ang;
            error_ang_pub_.publish(ang_error);

            // verbose
            if (verbose)
            {
                ROS_INFO(" TMOVE :  FV(%6.3f) AV(%6.7f)", cmd_lin_vel, cmd_ang_vel);
                ROS_INFO("[Pos] Current:(%6.3f,%6.3f), Target:(%6.3f,%6.3f)", pos_rbt.x, pos_rbt.y, target.x, target.y);
                ROS_WARN("[Error] Pos:(%6.3f), Ang:(%6.7f)", error_pos, error_ang);
            }

            // Record Data
            if (record) {
                data_file << ros::Time::now().toSec() << "," << error_pos << "," << pos_rbt.x << "," << cmd_lin_vel << "," << error_ang << "," << cmd_ang_vel << "\n";
            }

            // wait for rate
            rate.sleep();
        }
    }

    // attempt to stop the motors (does not work if ros wants to shutdown)
    msg_cmd.linear.x = 0;
    msg_cmd.angular.z = 0;
    pub_cmd.publish(msg_cmd);

    if (record) {data_file.close();}

    ROS_INFO(" TMOVE : ===== END =====");
    return 0;
}