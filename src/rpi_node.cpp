/* ------------------------------

The goal of this package should be:

Subscibe: (from hector mapping)
    /map_metadata (nav_msgs/MapMetaData)
        Get the map data from this topic, which is latched, and updated periodically.
    /map (nav_msgs/OccupancyGrid)
        Get the map data from this topic, which is latched, and updated periodically
    /slam_out_pose (geometry_msgs/PoseStamped)
        The estimated robot pose without covariance
    /poseupdate (geometry_msgs/PoseWithCovarianceStamped)
     

Publish:
    /cmd_vel for the arduino node




Notes:
hector mapping:
Subscribe:
    /scan (sensor_msgs/LaserScan):
        The laser scan used by the SLAM system.
    /syscommand (std_msgs/String):
        System command. If the string equals "reset" the map and robot pose are reset to their inital state.
Published Topics
    /map_metadata (nav_msgs/MapMetaData)
        Get the map data from this topic, which is latched, and updated periodically.
    /map (nav_msgs/OccupancyGrid)
        Get the map data from this topic, which is latched, and updated periodically
    /slam_out_pose (geometry_msgs/PoseStamped)
        The estimated robot pose without covariance
    /poseupdate (geometry_msgs/PoseWithCovarianceStamped)




------------------------------- */

#include "ros/ros.h"          // 加入ROS公用程序
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"

// #include "std_msgs/Int64.h"  // 所要publish的message header，在此是std_msgs package底下的String.msg
#include <sstream>
#include <iostream>

// 1. Publish Topic: /cmd_vel
/* 2. Subscribe Topic: 
    /map_metadata (nav_msgs/MapMetaData)
    /map (nav_msgs/OccupancyGrid)
    /slam_out_pose (geometry_msgs/PoseStamped) (w/o covariance)
    /poseupdate (geometry_msgs/PoseWithCovarianceStamped)
*/

ros::Publisher cmd_vel_pub;
geometry_msgs::Twist cmd_vel_msg;

ros::Subscriber map_meta_sub
ros::Subscriber map_sub
ros::Subscriber slam_out_pose_sub
ros::Subscriber poseupdate_sub

//////////// DON'T NEED THIS CALLBACK NOW ///////////////
void MUL_result_Callback(const sensor_msgs::Scan::ConstPtr& msg)
{
  ROS_INFO("message from Arduino: %ld", msg->data);
  // std_msgs::Int64 input_number;  // 建立暫存的message，先將資料存入此變數，再進行publish
    
    // static int num = 0;

    // input_number.data = num;   // 寫入msg message中的data欄位 
  std::cin>>input_number.data;
    // key : put "delay" before ros::spinOnce(); to allow Arduino to handle data and send back.
  RPi_pub.publish(input_number);
    // loop_rate.sleep(); 
  ROS_INFO("user's input is %ld", input_number.data); 
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "RPi");  //一開始必須先初始化，指定node名稱為talker

  /* 該node與ROS系統通訊的存取點(handle)，建構子會初始化該node，
     當離開此scope，解構子會關閉該node */
  ros::NodeHandle n;     

  /* advertise()會將建立topic的資訊告訴master node，回傳一個Publisher物件(在此為chatter_pub)
     之後可使用該物件的 publish() 方法進行publish
     而預計publish的message為std_msgs package的String.msg (std::msgs::String)   
     指定的topic名稱為chatter
     10指的是message queue，若publish太快，超過1000個message，新publish的message會被捨棄
  */ 
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  map_meta_sub = n.subscribe("map_metadata", 10, map_meta_Callback );
  map_sub = n.subscribe("map", 10, map_Callback );
  slam_out_pose_sub = n.subscribe("slam_out_pose", 10, slam_out_pose_Callback );
  poseupdate_sub = n.subscribe("poseupdate ", 10, poseupdate_Callback );

  ros::Rate loop_rate(2);   // 2Hz


  nh.spin();

/*
  // int count = 0;
  while (ros::ok())
  {
  // std_msgs::Int64 input_number;  // 建立暫存的message，先將資料存入此變數，再進行publish
    
    // static int num = 0;

    // input_number.data = num;   // 寫入msg message中的data欄位 
  
  ROS_INFO("Input Left PWM: "); 
  std::cin>>input_left_number.data;
  ROS_INFO("Input Right PWM: "); 
  std::cin>>input_right_number.data;
    // key : put "delay" before ros::spinOnce(); to allow Arduino to handle data and send back.
  Left_pub.publish(input_left_number);
  Right_pub.publish(input_right_number);
    // loop_rate.sleep(); 
  ROS_INFO("user's left input is %ld", input_left_number.data); 
  ROS_INFO("user's right input is %ld", input_right_number.data); 
    // ros::spinOnce();   // 呼叫一次 callback function，在subscriber才有用
    // num++;
  loop_rate.sleep(); 

  }
  // ros::spin();
 */ 
  
  // ASSERT NOT REACHED
  return 0;
}
