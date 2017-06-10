#ifndef POINT_KINEMATICS_H
#define POINT_KINEMATICS_H

#include <iostream>
#include <vector>
#include <string>
#include <math.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "tr5_simu/common.h"


class PointKinematics
{
private:
    // messages
    geometry_msgs::Twist command_vel_, desired_vel_;
    nav_msgs::Odometry pose_msg_;

    std::vector<double> path_x_;
    std::vector<double> path_y_;
    size_t iterator_;

    double linear_scape_vel_;
    double angular_scape_vel_;
    double turn_angular_vel_;

    double destiny_yaw_;
    double destiny_x_;
    double destiny_y_;

    double final_x_;
    double final_y_;

    bool found_last_objective_;
    bool found_objective_;
    bool need_update_orientation_;
    bool first_turning_;

    // callback
    double ori_x_;
    double ori_y_;
    double ori_yaw_;
    double new_linear_vel_;
    double new_angular_vel_;

    void updateResources();
    void updateDestinyPosAngle();


public:
    PointKinematics();
    ~PointKinematics();

    void dsrCallback(const geometry_msgs::Twist::ConstPtr& desired_vel){this->desired_vel_ = *desired_vel;}
    void odomCallback(const nav_msgs::Odometry::ConstPtr& pose_msg){this->pose_msg_ = *pose_msg;}

    void run();
    void cleanResources();

    void setLinVelScape(double vel){linear_scape_vel_ = vel;}
    void setAngVelScape(double vel){angular_scape_vel_ = vel;}


    void setDestinyVectorPositionsX(std::vector<double> vec_x){path_x_ = vec_x;}
    void setDestinyVectorPositionsY(std::vector<double> vec_y){path_y_ = vec_y;}
    void setNeedUpdateOrientation(bool value){ need_update_orientation_ = value;}
    void setFinalX(double x){final_x_ = x;}
    void setFinalY(double y){final_y_ = y;}
    void setFoundLastObjective(bool value){found_last_objective_ = value;}

    bool getFoundLastObjective() const {return found_last_objective_;}
    bool getNeedUpdateOrientation() const {return need_update_orientation_;}
    bool getFoundObjective() const {return found_objective_;}

    double getLinearVel() const {return new_linear_vel_;}
    double getAngularVel() const {return new_angular_vel_;}
};

#endif //POINT_KINEMATICS_H
