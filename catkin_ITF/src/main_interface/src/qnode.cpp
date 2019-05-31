/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include "../include/main_interface/qnode.hpp"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "sensor_msgs/image_encodings.h"//added head file
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace main_interface {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"main_interface");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
    chatter_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
    image_transport::ImageTransport it(n);
    image_sub = it.subscribe("GUI/face_result",100,&QNode::myCallback_img,this);
    //chatter_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"main_interface");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
    image_transport::ImageTransport it(n);
    image_sub = it.subscribe("GUI/face_result",100,&QNode::myCallback_img,this);
    chatter_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
    //chatter_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	start();
	return true;
}

void QNode::run() {
    while (!ros::ok() ) {
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown();
    // used to signal the gui for a shutdown (useful to roslaunch)
    }
    log(Info,"I'm running!");
    ros::spin();
}

void QNode::myCallback_img(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    img = cv_ptr->image;
    if (img.rows > 480){
        img = img.rowRange(img.rows/4,img.rows/4*3);
        img = img.colRange(img.cols/4,img.cols/4*3);
    }
    image = QImage(img.data,img.cols,img.rows,img.step[0],QImage::Format_RGB888);//change  to QImage format
    //image.re
    //ROS_INFO("I'm setting picture in mul_t callback function!");
    Q_EMIT loggingCamera();
  }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void QNode::runUp()
{
  ros::NodeHandle n;
  // Add your ros communications here.
  chatter_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
  //chatter_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  ros::Rate loop_rate(1);
  log(Info,std::string("I'm running Up!"));
  if( ros::ok() ) {
    geometry_msgs::Twist msg;
            msg.linear.x = 1.0;
            msg.angular.z = 0.0;
    chatter_publisher.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
void QNode::runDown()
{
  ros::NodeHandle n;
  // Add your ros communications here.
  chatter_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
  //chatter_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  ros::Rate loop_rate(1);
  log(Info,std::string("I'm running Down!"));
  if( ros::ok() ) {
    geometry_msgs::Twist msg;
            msg.linear.x = -1.0;
            msg.angular.z = 0.0;
    chatter_publisher.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
void QNode::runRight()
{
  ros::NodeHandle n;
  // Add your ros communications here.
  //chatter_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  chatter_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
  ros::Rate loop_rate(1);
  log(Info,std::string("I'm running Right!"));
  if ( ros::ok() ) {
    geometry_msgs::Twist msg;
            msg.linear.x = 0.0;
            msg.angular.z = -1.0;
    chatter_publisher.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
void QNode::runLeft()
{
  ros::NodeHandle n;
  // Add your ros communications here.
  //chatter_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  chatter_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);

  ros::Rate loop_rate(1);
  log(Info,std::string("I'm running Left!"));
  if ( ros::ok() ) {
    geometry_msgs::Twist msg;
            msg.linear.x = 0.0;
            msg.angular.z = 1.0;
    chatter_publisher.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
void QNode::stop()
{
  ros::NodeHandle n;
  chatter_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
  //chatter_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  log(Info,std::string("Stop Now!"));
  if ( ros::ok() ) {
    geometry_msgs::Twist msg;
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
    chatter_publisher.publish(msg);
    ros::spinOnce();
  }
}
void QNode::send_goal(double X, double Y)
{
     //tell the action client that we want to spin a thread by default
     MoveBaseClient ac("move_base", true);

     //wait for the action server to come up
     while(!ac.waitForServer(ros::Duration(5.0))){
       ROS_INFO("Waiting for the move_base action server to come up");
     }

     move_base_msgs::MoveBaseGoal goal;

     //we'll send a goal to the robot to move 1 meter forward
     goal.target_pose.header.frame_id = "map";
     goal.target_pose.header.stamp = ros::Time::now();

     goal.target_pose.pose.position.x = X;
     goal.target_pose.pose.position.y = Y;
     goal.target_pose.pose.orientation.w = 1.0;
     log(Info,std::string("Sending goal!"));
     log(Info,std::string("I'm moving!"));

     ac.sendGoal(goal);
     /*ac.waitForResult();
     if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
       log(Info,std::string("Hooray, I got the goal!"));
     else
     log(Info,std::string("I failed to move to send goal  meter for some reason!"));*/
}
void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace main_interface
