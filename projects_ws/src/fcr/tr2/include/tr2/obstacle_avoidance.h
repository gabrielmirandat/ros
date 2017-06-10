// Assumo que no início da simulação nao existe obstáculo frontal
// Adcionar tratamento se inicio com obs_front && obs_lados

// Com this->scan_msg_.ranges.size();
// Observou-se que o laser possui 720 leituras
// O laser passa a funcionar a partir de um certo número de iterações.

// this->scan_msg_ - armazena leituras do laser (sensor_msgs/LaserScan)
// this->sonar_msg_ - armazena leituras do sonar (p2os_msgs/SonarArray)
// this->odom_msg_ - armazena leituras da odometria (nav_msgs::Odometry)
// this->desired_vel_ - armazena leituras da velocidade (geometry_msgs/Twist)
// this->command_vel_ - mensagem de velocidade a ser publicada para correção (geometry_msgs/Twist)

// laser:: 720 pos - de 0 a 719 (0 - 359 e 360 - 719)
// http://docs.ros.org/jade/api/sensor_msgs/html/msg/LaserScan.html
// 720 medidas para um range de 270 graus = 0.375 angulos


#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

#include <vector>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

class ObstacleAvoidance
{
private:
    geometry_msgs::Twist desired_vel_;
    sensor_msgs::LaserScan scan_msg_;

    float center_total_count_;
    float lateral_total_right_count_;
    float lateral_total_left_count_;
    float left_total_count_;
    float right_total_count_;

    float center_count_;
    float lateral_right_count_;
    float lateral_left_count_;
    float left_count_;
    float right_count_;

    double lateral_left_minor_value_;
    double lateral_right_minor_value_;
    double left_total_sum_;
    double right_total_sum_;

    bool has_center_obs_;
    bool has_lateral_obs_;
    bool has_side_obs_;

    bool center_obstruction_;
    bool lateral_obstruction_;
    bool side_obstruction_;
    bool very_close_flag_;

    bool turning_;

    double linear_scape_vel_;
    double angular_scape_vel_;

    // callback
    double new_linear_vel_;
    double new_angular_vel_;

    double old_linear_vel_;
    double old_angular_vel_;
    bool use_old_vel_;



    void updateResources();
    void obtainLaserSamples(int laser_size);
    void updateVelocity();

public:
    ObstacleAvoidance();
    ~ObstacleAvoidance();

    void dsrCallback(const geometry_msgs::Twist::ConstPtr& desired_vel){this->desired_vel_ = *desired_vel;}
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser_msg){this->scan_msg_ = *laser_msg;}

    void run();

    bool getCenterObstruction() const {return center_obstruction_;}
    bool getSideObstruction() const {return side_obstruction_;}
    bool getLateralObstruction() const {return lateral_obstruction_;}
    bool getVeryCloseFlag() const {return very_close_flag_;}

    void setTurning(bool value){turning_ = value;}

    void setLinVelScape(double vel){linear_scape_vel_ = vel;}
    void setAngVelScape(double vel){angular_scape_vel_ = vel;}

    double getLinearVel() const {return new_linear_vel_;}
    double getAngularVel() const {return new_angular_vel_;}
};

#endif //OBSTACLE_AVOIDANCE_H
