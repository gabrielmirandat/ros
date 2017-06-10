#include "tr3/obstacle_avoidance.h"

// -- public

ObstacleAvoidance::ObstacleAvoidance()
{
    center_total_count_ = lateral_total_right_count_ = lateral_total_left_count_ = 0.0;
    left_total_count_ = right_total_count_ = 0.0;

    center_count_ = lateral_right_count_ = lateral_left_count_ = 0.0;
    left_count_ = right_count_ = 0.0;

    lateral_left_minor_value_ = lateral_right_minor_value_ = 30.0;
    left_total_sum_ = right_total_sum_ = 0.0;

    has_center_obs_ = has_lateral_obs_ = has_side_obs_ = false;
    center_obstruction_ = lateral_obstruction_ = side_obstruction_ = very_close_flag_ = false;

    turning_ = false;

    new_linear_vel_ = new_angular_vel_ = 0.0;

    old_linear_vel_ = old_angular_vel_ = 0.0;

    use_old_vel_ = false;
}

ObstacleAvoidance::~ObstacleAvoidance()
{
}

// -- private

void ObstacleAvoidance::updateResources()
{
    new_linear_vel_ = desired_vel_.linear.x;
    new_angular_vel_ = desired_vel_.angular.z;

    center_total_count_ = lateral_total_right_count_ = lateral_total_left_count_ = 0.0;
    left_total_count_ = right_total_count_ = 0.0;

    center_count_ = lateral_right_count_ = lateral_left_count_ = 0.0;
    left_count_ = right_count_ = 0.0;

    lateral_left_minor_value_ = lateral_right_minor_value_ = 30.0;
    left_total_sum_ = right_total_sum_ = 0.0;

    has_center_obs_ = has_lateral_obs_ = has_side_obs_ = false;
    center_obstruction_ = lateral_obstruction_ = side_obstruction_ = very_close_flag_ = false;

}

void ObstacleAvoidance::obtainLaserSamples(int laser_size)
{

    double close_min_dist;
    double center_min_dist = 0.8;
    double lateral_min_dist = 0.7;
    double side_min_dist = 0.6;

    double center_percent = 0.01;
    double lateral_percent = 0.01;
    double side_percent = 0.01;

    for(int i= laser_size/2 -  laser_size/4; i< laser_size/2 + laser_size/4; i+=2)
    {

        if(i > (laser_size/2 - laser_size/8) && i< (laser_size/2 + laser_size/8))
        {
            // ROS_INFO("[OBS_AVOID] center_total_count_!!! %d", center_total_count_);
            close_min_dist = 0.25;
            if(scan_msg_.ranges[i] < close_min_dist) very_close_flag_ = true;

            center_total_count_+= 1.0;
            if(scan_msg_.ranges[i] < center_min_dist) center_count_+= 1.0;
        }
        else if(i < (laser_size/2 - laser_size/8))
        {
            // ROS_INFO("[OBS_AVOID] lateral_total_right_count_!!! %d", lateral_total_right_count_);
            close_min_dist = 0.3;
            if(scan_msg_.ranges[i] < close_min_dist) very_close_flag_ = true;

            lateral_total_right_count_+= 1.0;

            if((scan_msg_.ranges[i] > 0.0) && (scan_msg_.ranges[i] < lateral_right_minor_value_))
                lateral_right_minor_value_ = scan_msg_.ranges[i];

            if(scan_msg_.ranges[i] < lateral_min_dist) lateral_right_count_+= 1.0;
        }
        else if(i > (laser_size/2 + laser_size/8))
        {
            // ROS_INFO("[OBS_AVOID] lateral_total_left_count_!!! %d", lateral_total_left_count_);
            close_min_dist = 0.3;
            if(scan_msg_.ranges[i] < close_min_dist) very_close_flag_ = true;

            lateral_total_left_count_+= 1.0;

            if((scan_msg_.ranges[i] > 0.0) && (scan_msg_.ranges[i] < lateral_left_minor_value_))
                lateral_left_minor_value_ = scan_msg_.ranges[i];

            if(scan_msg_.ranges[i] < lateral_min_dist) lateral_left_count_+= 1.0;
        }
    }

    for(int i=0; i< laser_size/4; i+=6)
    {
        // ROS_INFO("[OBS_AVOID] right_total_count_!!! %d", right_total_count_);
        close_min_dist = 0.4;
        if(scan_msg_.ranges[i] < close_min_dist) very_close_flag_ = true;

        right_total_count_+= 1.0;
        right_total_sum_+= scan_msg_.ranges[i];
        if(scan_msg_.ranges[i] < side_min_dist) right_count_+= 1.0;
    }

    for(int i= laser_size * 3/4 ; i< laser_size; i+=6)
    {
        close_min_dist = 0.4;
        if(scan_msg_.ranges[i] < close_min_dist) very_close_flag_ = true;

        left_total_count_+=1.0;
        left_total_sum_+= scan_msg_.ranges[i];
        if(scan_msg_.ranges[i] < side_min_dist) left_count_+1.0;
    }


    if((center_count_ / center_total_count_) >= center_percent) has_center_obs_ = true;
    if((lateral_right_count_ / lateral_total_right_count_) >= lateral_percent) has_lateral_obs_ = true;
    if((lateral_left_count_ / lateral_total_left_count_) >= lateral_percent) has_lateral_obs_ = true;
    if((right_count_ / right_total_count_) >= side_percent) has_side_obs_ = true;
    if((left_count_ / left_total_count_) >= side_percent) has_side_obs_ = true;

    // if(center_count_ > 0)  has_center_obs_ = true;
    // if(lateral_right_count_ > 0) has_lateral_obs_ = true;
    // if(lateral_left_count_ > 0) has_lateral_obs_ = true;
    // if(right_count_ > 0) has_side_obs_ = true;
    // if(left_count_ > 0) has_side_obs_ = true;

}

void ObstacleAvoidance::updateVelocity()
{
    new_linear_vel_ = linear_scape_vel_;

    if(very_close_flag_)
    {
        ROS_INFO("[OBS_AVOID] Very close obstruction!!!");

        new_linear_vel_ = 0.0;
        new_angular_vel_ = 0.0;
    }
    else if(has_center_obs_)
    {
        ROS_INFO("[OBS_AVOID] Center obstruction!!!");

        center_obstruction_ = true;
        new_linear_vel_ = 0.0;  // -0.8 : 0.8

        if(!turning_) // começa a girar
        {
            double left = lateral_left_minor_value_;
            double right = lateral_right_minor_value_;

            new_angular_vel_ = (left <= right) ? -1.0 * angular_scape_vel_ : angular_scape_vel_;  // -0.8 : 0.8
            old_angular_vel_ = new_angular_vel_;

            turning_ = true;
        }
        else  // if(turning)
        {
            new_angular_vel_ = old_angular_vel_;
        }
    }
    else if(has_lateral_obs_)
    {
        ROS_INFO("[OBS_AVOID] Lateral obstruction!!!");

        lateral_obstruction_ = true;
        new_linear_vel_ = 0.07;

        if(!turning_) // começa a girar
        {
            double left = lateral_left_minor_value_;
            double right = lateral_right_minor_value_;

            new_angular_vel_ = (left <= right) ? -1.0 * angular_scape_vel_ : angular_scape_vel_;  // -0.8 : 0.8
            old_angular_vel_ = new_angular_vel_;

            turning_ = true;
        }
        else  // if(turning)
        {
            new_angular_vel_ = old_angular_vel_;
        }
    }
    else
    {
        turning_ = false;

        if(has_side_obs_)
        {
           ROS_INFO("[OBS_AVOID] Side obstruction!!!");

           side_obstruction_ = true;
           new_angular_vel_ = (left_total_sum_ <= right_total_sum_) ? -1.0 * angular_scape_vel_ : angular_scape_vel_;  // -0.8 : 0.8
        }
    }
}

void ObstacleAvoidance::run()
{
    updateResources();

    int laser_size = scan_msg_.ranges.size();
    if(laser_size > 0)
    {
        obtainLaserSamples(laser_size);
        updateVelocity();
    }
}
