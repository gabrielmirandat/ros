#include "ros/ros.h"
#include "std_msgs/Float32.h"

// $ rostopic pub -1 /v_left std_msgs/Float32 -- 0.5
// $ rostopic pub -1 /v_right std_msgs/Float32 -- 0.5

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pioneer_circle_quad");
  ros::NodeHandle node_handle_vleft;
  ros::NodeHandle node_handle_vright;

  ros::Publisher pioneer_circle_quad_vleft_pub = node_handle_vleft.advertise<std_msgs::Float32>("v_left", 10);
  ros::Publisher pioneer_circle_quad_vright_pub = node_handle_vright.advertise<std_msgs::Float32>("v_right", 10);

  // std_msgs::Float32 msg;
  // msg.data = 0.5;
  
  // pioneer_circle_quad_vleft_pub.publish(msg);
  // pioneer_circle_quad_vright_pub.publish(msg);
  
  std_msgs::Float32 msg_vleft, msg_vright;
  ros::Rate loop_rate(10);
  int count;

  // circle
  count = 0;
  while (ros::ok())
  {
    if(count == 115) break;

    msg_vleft.data = 0.5;
    msg_vright.data = 0.2;

    ROS_INFO("circle: %f", msg_vleft.data);
    ROS_INFO("circle: %f", msg_vright.data);

    pioneer_circle_quad_vleft_pub.publish(msg_vleft);
    pioneer_circle_quad_vright_pub.publish(msg_vright);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  // first side
  count = 0;
  while (ros::ok())
  {
    if(count == 40) break;

    msg_vleft.data = 0.5;
    msg_vright.data = 0.5;

    ROS_INFO("first: %f", msg_vleft.data);
    ROS_INFO("first: %f", msg_vright.data);

    pioneer_circle_quad_vleft_pub.publish(msg_vleft);
    pioneer_circle_quad_vright_pub.publish(msg_vright);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  // turn
  count = 0;
  while (ros::ok())
  {
    if(count == 10) break;

    msg_vleft.data = 0.5;
    msg_vright.data = 0.0;

    ROS_INFO("turn: %f", msg_vleft.data);
    ROS_INFO("turn: %f", msg_vright.data);

    pioneer_circle_quad_vleft_pub.publish(msg_vleft);
    pioneer_circle_quad_vright_pub.publish(msg_vright);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  // second side
  count = 0;
  while (ros::ok())
  {
    if(count == 40) break;

    msg_vleft.data = 0.5;
    msg_vright.data = 0.5;

    ROS_INFO("second: %f", msg_vleft.data);
    ROS_INFO("second: %f", msg_vright.data);

    pioneer_circle_quad_vleft_pub.publish(msg_vleft);
    pioneer_circle_quad_vright_pub.publish(msg_vright);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  // turn
  count = 0;
  while (ros::ok())
  {
    if(count == 10) break;

    msg_vleft.data = 0.5;
    msg_vright.data = 0.0;

    ROS_INFO("turn: %f", msg_vleft.data);
    ROS_INFO("turn: %f", msg_vright.data);

    pioneer_circle_quad_vleft_pub.publish(msg_vleft);
    pioneer_circle_quad_vright_pub.publish(msg_vright);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  // third side
  count = 0;
  while (ros::ok())
  {
    if(count == 40) break;

    msg_vleft.data = 0.5;
    msg_vright.data = 0.5;

    ROS_INFO("third: %f", msg_vleft.data);
    ROS_INFO("third: %f", msg_vright.data);

    pioneer_circle_quad_vleft_pub.publish(msg_vleft);
    pioneer_circle_quad_vright_pub.publish(msg_vright);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  // turn
  count = 0;
  while (ros::ok())
  {
    if(count == 10) break;

    msg_vleft.data = 0.5;
    msg_vright.data = 0.0;

    ROS_INFO("turn: %f", msg_vleft.data);
    ROS_INFO("turn: %f", msg_vright.data);

    pioneer_circle_quad_vleft_pub.publish(msg_vleft);
    pioneer_circle_quad_vright_pub.publish(msg_vright);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  // fourth side
  count = 0;
  while (ros::ok())
  {
    if(count == 40) break;

    msg_vleft.data = 0.5;
    msg_vright.data = 0.5;

    ROS_INFO("fourth: %f", msg_vleft.data);
    ROS_INFO("fourth: %f", msg_vright.data);

    pioneer_circle_quad_vleft_pub.publish(msg_vleft);
    pioneer_circle_quad_vright_pub.publish(msg_vright);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  // turn
  count = 0;
  while (ros::ok())
  {
    if(count == 10) break;

    msg_vleft.data = 0.5;
    msg_vright.data = 0.0;

    ROS_INFO("turn: %f", msg_vleft.data);
    ROS_INFO("turn: %f", msg_vright.data);

    pioneer_circle_quad_vleft_pub.publish(msg_vleft);
    pioneer_circle_quad_vright_pub.publish(msg_vright);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  // stop
  msg_vleft.data = 0.0;
  msg_vright.data = 0.0;

  ROS_INFO("stop: %f", msg_vleft.data);
  ROS_INFO("stop: %f", msg_vright.data);

  pioneer_circle_quad_vleft_pub.publish(msg_vleft);
  pioneer_circle_quad_vright_pub.publish(msg_vright);

  return 0;
}