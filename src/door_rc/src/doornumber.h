#ifndef DOORNUMBER_H
#define DOORNUMBER_H
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>
#include <algorithm>
#include <sstream>
//dealdta
#include <opencv2/opencv.hpp>
#include <sys/types.h>
#include <iostream>
#include <dirent.h>
#include <unistd.h>
#include <stdio.h>
#include <vector>
#include <time.h>

using namespace cv;
using namespace std;
using namespace ml;
namespace Door_recognition {

class DoorNumber
{
public:
    DoorNumber();
    ~DoorNumber();
     void ImageCb(const sensor_msgs::ImageConstPtr& msg);
     void process_image(const cv::Mat &img);

private:
     string dir_path = "/home/sun/catkin_ITF/src/door_rc/data1/sample/";
     void samplePath(vector<string> &sample_path, vector<int> &labels);
     void samplePath(string dirPath, vector<string> &sample_path);
    Mat getHogMat(string path);
    Mat gotHogMat(Mat src);

   void KNN_train();
   int predict(Mat inputImage);
   Mat deal_camera(Mat srcImage);

   Mat perspectivetransfer(Mat srcImage,int max_id);
   void referenceline(Mat srcImage,Point2f corner_point[]);
   //void computecornerpoint(int max_id, Point2f pts_src[]);
   float computedistance(int height);
   Point2f computecrosspoint(float r1,float r2);
   bool judge_door_number_center(float dis, Point2f p_0, Point2f p_2);
   ros::NodeHandle nh_;
   image_transport::ImageTransport it_;
   image_transport::Subscriber image_sub_;
   image_transport::Publisher image_pub_;
   ros::Publisher result_pub;
   ros::Publisher cmd_pub;

   Mat trainImage;//用于存放训练图
   Mat labelImage;//用于存放标签
   Ptr<KNearest> knn;
   int result;//predict result with knn
   vector<vector<Point> > contours;//点容器的容器，用于存放轮廓的点的容器的容器
   Point positiosn;
};
}  // namespace Door_recognition
#endif // DOORNUMBER_H
