#ifndef TRABALHO_H
#define TRABALHO_H

#include <printf.h>
#include <vector>
#include <string>
#include <assert.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

#include "graph.h"
#include "obstacle_avoidance.h"
#include "occupancy_grid.h"
#include "point_kinematics.h"
#include "topological_map.h"

#include "tr5_zrobot/common.h"

class Trabalho
{

private:
    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    ros::Subscriber dsr_sub_, laser_sub_, odom_sub_;

    geometry_msgs::Twist command_vel_, desired_vel_;
    nav_msgs::Odometry pose_msg_;

    void dsrCallback(const geometry_msgs::Twist::ConstPtr& desired_vel);
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser_msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& pose_msg);

    Graph* graph_;
    ObstacleAvoidance* obs_avoid_;
    OccupancyGrid* occ_grid_;
    PointKinematics* point_kin_;
    TopologicalMap* topo_map_;

    double choosen_linear_vel_, choosen_angular_vel_;
    double initial_pos_x_, initial_pos_y_;
    double base_pos_x_, base_pos_y_;
    double final_pos_x_, final_pos_y_;
    int timer_force_corretion_counter_;
    int obstacle_force_corretion_counter_;
    bool obstacle_can_count_;

    int current_robot_node_;
    int last_robot_node_;

    void cleanResources();

public:
    Trabalho(ros::NodeHandle nh);
    ~Trabalho();

    void spinTrab1();
    void spinNav();
    void spinTrab2();
    void spinMap();
    void spinTrab3();
    void spinTrab5();


};

#endif // TRABALHO_H
