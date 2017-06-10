#include <vector>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <p2os_msgs/SonarArray.h>

class ObstacleAvoidance
{
private:
    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    ros::Subscriber laser_sub_, sonar_sub_, dsr_sub_;

    geometry_msgs::Twist command_vel_, desired_vel_;
    sensor_msgs::LaserScan scan_msg_;
    p2os_msgs::SonarArray sonar_msg_;

    // my variables
    int global_counter_;                        ///< Global vector to wait for laser stabilization.
    std::vector<float> left_laser_scan_;        ///< Samples for left laser scan.
    std::vector<float> right_laser_scan_;       ///< Samples for right laser scan.

    bool last_action_;
    bool has_left_obs_;
    bool has_right_obs_;
    double left_sum_;
    double right_sum_;
    float scape_vel_;



    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser_msg);
    void sonarCallback(const p2os_msgs::SonarArray::ConstPtr& sonar_msg);
    void dsrCallback(const geometry_msgs::Twist::ConstPtr& desired_vel);

    void algorithm();

public:
    ObstacleAvoidance(ros::NodeHandle nh);
    ~ObstacleAvoidance();

    void spin();
};
