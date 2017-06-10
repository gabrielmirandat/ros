#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"

// $ rostopic pub -1 /v_left std_msgs/Float32 -- 0.5
// $ rostopic pub -1 /v_right std_msgs/Float32 -- 0.5

ros::Rate loop_rate(10);
std_msgs::Float32 msg_vleft, msg_vright;
sensor_msgs::LaserScan msg_laser;
int global_counter;

void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  if(global_counter == 10)
  {  
    global_counter = 0;
    if(msg->ranges[msg->ranges.size()] < 3.0)
    {
      while(ros::ok())
      {
           
        ros::spinOnce();
        loop_rate.sleep();
      }
    }

  }else
  {
    global_counter++;
  }

  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  global_counter = 0;

  ros::init(argc, argv, "pioneer_circle_quad");
  ros::NodeHandle node_handle_vleft;
  ros::NodeHandle node_handle_vright;
  ros::NodeHandle node_handle_laser;

  // publishers
  ros::Publisher vleft_pub = node_handle_vleft.advertise<std_msgs::Float32>("v_left", 10);
  ros::Publisher vright_pub = node_handle_vright.advertise<std_msgs::Float32>("v_right", 10);
  
  // subscribers
  ros::Subscriber laser_sub = node_handle_laser.subscribe("hokuyo_scan", 10, chatterCallback);
  
  while (ros::ok())
  {
    // ROS_INFO("circle: %f", msg_vright.data);

    msg_vleft.data = 0.5;
    msg_vright.data = 0.5;

    vleft_pub.publish(msg_vleft);
    vright_pub.publish(msg_vright);

    ros::spinOnce();

    loop_rate.sleep();
  }
}