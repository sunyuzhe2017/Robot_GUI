/**
 * @file /include/main_interface/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef main_interface_QNODE_HPP_
#define main_interface_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
//added for image_view 2018.6.7
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <QImage>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace main_interface {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
    QNode(int argc, char** argv );
    virtual ~QNode();
    bool init();
    bool init(const std::string &master_url, const std::string &host_url);
    void run();
    void stop();
    void runUp();
    void runDown();
    void runRight();
    void runLeft();
    void send_goal(double x,double y);//set map position!
	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

    QStringListModel* loggingModel() { return &logging_model; }
    void log( const LogLevel &level, const std::string &msg);

    void myCallback_img(const sensor_msgs::ImageConstPtr& msg);//camera callback function
    QImage image;

Q_SIGNALS:
    void loggingUpdated();
    void rosShutdown();
    void loggingCamera();//发出设置相机图片信号
private:
    int init_argc;
    char** init_argv;
    ros::Publisher chatter_publisher;
    QStringListModel logging_model;
    image_transport::Subscriber image_sub;
    cv::Mat img;
};

}  // namespace main_interface

#endif /* main_interface_QNODE_HPP_ */
