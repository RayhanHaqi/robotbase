#include "ros/ros.h"
#include "rs232.h"
#include "stdlib.h"
#include "stdio.h"
#include <string.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Pose2D.h"
#include <iostream>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

// create publisher odometry
// ros::Publisher odom_pub;


typedef enum
{

  segitiga = 0,
  bulat = 1,
  eks = 2,
  r1 = 5,
  l2 = 6

} stik_button;

typedef struct
{

  uint8_t button[17];
  int axis[4];
  uint8_t prev_button[17];

} stik_t;

stik_t myControler;

short int robotSpeed[3] = {0, 0, 0};

float posisiOdom[3];
float offsetPos[3];
char bitLamp;

ros::Timer Thread_SerialTransmit;
ros::Timer Thread_SerialReceived;

ros::Subscriber subButton;
ros::Subscriber subAxis;

ros::Publisher odom_pub;

std_msgs::Int32 stik_Button;
std_msgs::Int16MultiArray stik_Axis;

// cport_nr=16,  USB0

int i, n,
    cport_nr = 16,
    bdrate = 115200;

unsigned char buf[4096];

int status_control;

void SerialTransmitEvent(const ros::TimerEvent &event);
void SerialReceiveEvent(const ros::TimerEvent &event);
void CallbackButton(const std_msgs::Int32 &msg_btn);
void CallbackAxis(const std_msgs::Int16MultiArray &msg_Axis);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lol");

  ros::NodeHandle nh;
  ros::MultiThreadedSpinner mts;
  ros::Rate RosRate(10);

  // create publisher odometry
  // odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

  stik_Button.data = 4095;

  for (int i = 0; i < 12; i++)
  {
    myControler.button[i] = (stik_Button.data >> i) & 1;
  }

  char mode[] = {'8', 'N', '1', 0};

  if (RS232_OpenComport(16, 115200, mode))
  {
    printf("Cannot Open COM Port\n");

    return (0);
  }
  else
  {
    printf("Port Open\n");
  }

  subButton = nh.subscribe("controller/button", 16, CallbackButton);
  subAxis = nh.subscribe("controller/axis", 6, CallbackAxis);

  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

  Thread_SerialTransmit = nh.createTimer(ros::Duration(0.1), SerialTransmitEvent);
  Thread_SerialReceived = nh.createTimer(ros::Duration(0.01), SerialReceiveEvent);
  mts.spin();
}

void SerialTransmitEvent(const ros::TimerEvent &event)
{

  if (myControler.button[l2])
  {
    robotSpeed[0] = myControler.axis[0] / 800;
    robotSpeed[1] = -1 * myControler.axis[1] / 800;
    robotSpeed[2] = myControler.axis[3] / 1700;
  }
  else
  {
    robotSpeed[0] = myControler.axis[0] / 500;
    robotSpeed[1] = -1 * myControler.axis[1] / 500;
    robotSpeed[2] = myControler.axis[3] / 1300;
  }

  if (myControler.button[segitiga] == 0 && myControler.prev_button[segitiga] == 1)
  {
    status_control ^= 1;
  }

  myControler.prev_button[segitiga] = myControler.button[segitiga];

  if (myControler.button[bulat] == 0 && myControler.prev_button[bulat] != 0)
  {
    offsetPos[0] = posisiOdom[0] + offsetPos[0];
    offsetPos[1] = posisiOdom[1] + offsetPos[1];
    offsetPos[2] = posisiOdom[2] + offsetPos[2];
  }

  myControler.prev_button[bulat] = myControler.button[bulat];

  if (myControler.button[eks] == 0)
  {
    offsetPos[0] = 0;
    offsetPos[1] = 0;
    offsetPos[2] = 0;
  }

  char data_kirim[23] = {'m', 'r', 'i'};

  // robotSpeed[0]=;
  // printf(" Speed_x = %d", robotSpeed[0]);
  // printf("\n");
  // printf(" Speed_y = %d", robotSpeed[1]);
  // printf("\n");
  // printf(" Speed_z = %d", robotSpeed[2]);
  // printf("\n");
  // offsetPos[2]=42.92;
  memcpy(data_kirim + 3, &robotSpeed[0], 2);
  memcpy(data_kirim + 5, &robotSpeed[1], 2);
  memcpy(data_kirim + 7, &robotSpeed[2], 2);
  // status_control = 1;
  //  bitLamp = 0b111111101;
  data_kirim[9] = bitLamp;
  data_kirim[10] = status_control;
  memcpy(data_kirim + 11, &offsetPos[0], 4);
  memcpy(data_kirim + 15, &offsetPos[1], 4);
  memcpy(data_kirim + 19, &offsetPos[2], 4);

  RS232_SendBuf(cport_nr, (unsigned char *)data_kirim, 23);

  // std::cout << "posisi x = " << posisiOdom[0] << '\n'
  //           << "posisi y = " << posisiOdom[1] << '\n'
  //           << "rotasi z = " << posisiOdom[2] << '\n'
  //           << "status control = " << status_control << std::endl;
  // ROS_INFO("INFO");
  // ros::Time current_time = ros::Time::now();
  // tf::TransformBroadcaster odom_broadcaster;

  // // since all odometry is 6DOF we'll need a quaternion created from yaw
  // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(posisiOdom[2]);

  // // first, we'll publish the transform over tf
  // geometry_msgs::TransformStamped odom_trans;
  // odom_trans.header.stamp = current_time;
  // odom_trans.header.frame_id = "odom";
  // odom_trans.child_frame_id = "base_link";

  // odom_trans.transform.translation.x = posisiOdom[0];
  // odom_trans.transform.translation.y = posisiOdom[1];
  // odom_trans.transform.translation.z = 0.0;
  // odom_trans.transform.rotation = odom_quat;

  // // send the transform
  // odom_broadcaster.sendTransform(odom_trans);

  // next, we'll publish the odometry message over ROS
  //  nav_msgs::Odometry odom;
  //  odom.header.stamp = current_time;
  //  odom.header.frame_id = "odom";

  // //set the position
  // odom.pose.pose.position.x = posisiOdom[0];
  // odom.pose.pose.position.y = posisiOdom[1];
  // odom.pose.pose.position.z = 0.0;
  // odom.pose.pose.orientation = odom_quat;

  // //set the velocity
  // odom.child_frame_id = "base_link";
  // odom.twist.twist.linear.x = 0.1;
  // odom.twist.twist.linear.y = 0.1;
  // odom.twist.twist.angular.z = 0;

  // publish the message
}
void SerialReceiveEvent(const ros::TimerEvent &event)
{

  n = RS232_PollComport(cport_nr, buf, 4095);

  if (n > 10)
  {
    buf[n] = 0; /* always put a "null" at the end of a string! */

    for (i = 0; i < n; i++)
    {
      if (buf[i] < 32) /* replace unreadable control-codes by dots */
      {
        buf[i] = 0;
      }
    }

    std::string sLocal[5];
    char *token = strtok((char *)buf, ",");
    int data_len = 0;
    while (token != NULL)
    {

      sLocal[data_len].assign(token);

      token = strtok(NULL, ",");
      data_len++;
    }
    if (data_len == 4 && sLocal[0].length() > 3)
    {
      posisiOdom[0] = std::stof(sLocal[0]);
      posisiOdom[1] = std::stof(sLocal[1]);
      posisiOdom[2] = std::stof(sLocal[2]);

      ros::Time current_time = ros::Time::now();
      tf::TransformBroadcaster odom_broadcaster;

      // since all odometry is 6DOF we'll need a quaternion created from yaw
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(posisiOdom[2]);

      // first, we'll publish the transform over tf
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";

      odom_trans.transform.translation.x = posisiOdom[0];
      odom_trans.transform.translation.y = posisiOdom[1];
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;

      // send the transform
      odom_broadcaster.sendTransform(odom_trans);

      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";

      // set the position
      odom.pose.pose.position.x = posisiOdom[0];
      odom.pose.pose.position.y = posisiOdom[1];
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;

      // set the velocity
      odom.child_frame_id = "base_link";

      odom_pub.publish(odom);



    }
  }
}

void CallbackButton(const std_msgs::Int32 &msg_btn)
{

  stik_Button = msg_btn;

  for (int i = 0; i < 12; i++)
  {
    myControler.button[i] = (stik_Button.data >> i) & 1;
  }
}
void CallbackAxis(const std_msgs::Int16MultiArray &msg_Axis)
{
  stik_Axis = msg_Axis;

  myControler.axis[0] = msg_Axis.data.at(0);
  myControler.axis[1] = msg_Axis.data.at(1);
  myControler.axis[2] = msg_Axis.data.at(2);
  myControler.axis[3] = msg_Axis.data.at(3);
}
