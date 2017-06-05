#include "tr1/obstacle_avoidance.h"


ObstacleAvoidance::ObstacleAvoidance(ros::NodeHandle nh)
: nh_(nh), global_counter_(0),
  last_action_(false), has_left_obs_(false), has_right_obs_(false),
  left_sum_(0.0), right_sum_(0.0), scape_vel_(0.0)
{
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    laser_sub_ = nh_.subscribe("hokuyo_scan", 10, &ObstacleAvoidance::laserCallback, this);
    dsr_sub_ = nh_.subscribe("desired_vel", 10, &ObstacleAvoidance::dsrCallback, this);
    sonar_sub_ = nh_.subscribe("sonar", 10, &ObstacleAvoidance::sonarCallback, this);
}

void ObstacleAvoidance::laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser_msg)
{
    this->scan_msg_ = *laser_msg;
}

void ObstacleAvoidance::dsrCallback(const geometry_msgs::Twist::ConstPtr& desired_vel)
{
    this->desired_vel_ = *desired_vel;
}

void ObstacleAvoidance::sonarCallback(const p2os_msgs::SonarArray::ConstPtr& sonar_msg)
{
    this->sonar_msg_ = *sonar_msg;
}

void ObstacleAvoidance::algorithm()
{
    double new_linear_vel = desired_vel_.linear.x;
    double new_angular_vel = desired_vel_.angular.z;

    // Escreva aqui seu codigo
    
    // this->scan_msg_ - armazena leituras do laser (sensor_msgs/LaserScan)
    // this->sonar_msg_ - armazena leituras do sonar (p2os_msgs/SonarArray)
    // this->desired_vel_ - armazena leituras da velocidade (geometry_msgs/Twist)
    // this->command_vel_ - mensagem de velocidade a ser publicada para correção (geometry_msgs/Twist)

    // 720 pos - de 0 a 719
    // 0 - 359
    // 360 - 719
    int laser_size = scan_msg_.ranges.size();

    left_laser_scan_.clear();
    right_laser_scan_.clear();
    has_left_obs_ = has_right_obs_ = false;
    left_sum_ = right_sum_ = 0.0;

    if(laser_size > 0)
    {

        // get laser samples
        for(int i=0; i< laser_size/4; i+=12)
        {
            left_laser_scan_.push_back(scan_msg_.ranges[i]);

            if(scan_msg_.ranges[i] < 0.8) has_left_obs_ = true;
            left_sum_ += scan_msg_.ranges[i];
        }

        for(int i= laser_size/4; i< laser_size/2; i+=8)
        {
            left_laser_scan_.push_back(scan_msg_.ranges[i]);

            if(scan_msg_.ranges[i] < 1.5) has_left_obs_ = true;
            left_sum_ += scan_msg_.ranges[i];
        }

        for(int i= laser_size/2; i< laser_size*3/4; i+=8)
        {
            right_laser_scan_.push_back(scan_msg_.ranges[i]);

            if(scan_msg_.ranges[i] < 1.5) has_right_obs_ = true;
            right_sum_ += scan_msg_.ranges[i];
        }

        for(int i= laser_size*3/4; i< laser_size; i+=12)
        {
            right_laser_scan_.push_back(scan_msg_.ranges[i]);

            if(scan_msg_.ranges[i] < 0.8) has_right_obs_ = true;
            right_sum_ += scan_msg_.ranges[i];
        }

        scape_vel_ = (new_linear_vel > new_angular_vel)? new_linear_vel : new_angular_vel;

        // take action
        if(has_left_obs_ || has_right_obs_)
        {
            // turn right
            new_linear_vel = 0.0;
            new_angular_vel = scape_vel_;
            last_action_ = true;
        }
        else
        {
            if(last_action_ && new_linear_vel == 0.0)
            {
                new_linear_vel = scape_vel_;
                new_angular_vel = 0.0;
                last_action_ = false;
            }
        }
    }

    ROS_INFO("linear: %f", new_linear_vel);
    ROS_INFO("angular: %f\n\n", new_angular_vel);

    command_vel_.linear.x = new_linear_vel;
    command_vel_.angular.z = new_angular_vel;
}

void ObstacleAvoidance::spin()
{
    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        ros::spinOnce();
        algorithm();
        vel_pub_.publish(command_vel_);
        loop_rate.sleep();
    }

    command_vel_.linear.x = 0.0;
    command_vel_.angular.z = 0.0;
    vel_pub_.publish(command_vel_);
}

ObstacleAvoidance::~ObstacleAvoidance()
{
}
