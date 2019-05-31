#include "doornumber.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include "sensor_msgs/image_encodings.h"
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace enc = sensor_msgs::image_encodings;
namespace Door_recognition {

DoorNumber::DoorNumber():it_(nh_)
{
    image_pub_ = it_.advertise("camera/process", 1);
    result_pub = nh_.advertise<std_msgs::String>("door_result",1);
    cmd_pub = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);;

    /*订阅主题camera/image*/
    image_sub_ = it_.subscribe("camera/image", 1, &DoorNumber::ImageCb, this);
    KNN_train();
}

DoorNumber::~DoorNumber(){}

void DoorNumber::ImageCb(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
     cv::Mat img;
     /*转化成CVImage*/
     cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
     process_image(cv_ptr->image);
     //cv::imshow("camera/process",cv_ptr->image);
     //cv::waitKey(30);
}
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception is %s", e.what());
        return;
    }

    //cv::imshow(IN_WINDOW, cv_ptr->image);

}

void DoorNumber::process_image(const cv::Mat &img)
{
    if(img.rows > 60 && img.cols > 60)
    cv::circle(img, cv::Point(50,50), 10, CV_RGB(255,0,0));
    cv::Mat img_out;
    img_out = deal_camera(img);
    /*转化成ROS图像msg发布到主题out*/
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_out).toImageMsg();
    image_pub_.publish(msg);
}

void DoorNumber::samplePath(vector<string> &sample_path, vector<int> &labels)
{
  DIR    *dir;
  struct    dirent    *ptr;

  for (int i = 0; i < 10; ++i)
  {
    char path[1024];
    sprintf(path, "%s%d/.", dir_path.c_str(), i);
    dir = opendir(path); ///open the dir
    //读取目录下的所有文件
    while ((ptr = readdir(dir)) != NULL) ///read the list of this dir
    {
      char tmp[1024];

      if (ptr->d_name != string(".") && ptr->d_name != string(".."))
      {
        sprintf(tmp, "%s%d/%s", dir_path.c_str(), i, ptr->d_name);
        sample_path.push_back(tmp);
        labels.push_back(i);
      }

    }

    closedir(dir);
  }
}

void DoorNumber::samplePath(string dirPath, vector<string> &sample_path)
{
  DIR    *dir;
  struct    dirent    *ptr;

  char path[1024];
  sprintf(path, "%s.", dirPath.c_str());

  dir = opendir(path);

  while ((ptr = readdir(dir)) != NULL)
  {
    char tmp[1024];

    if (ptr->d_name != string(".") && ptr->d_name != string(".."))
    {
      sprintf(tmp, "%s%s", dir_path.c_str(), ptr->d_name);
      sample_path.push_back(tmp);
    }

  }

  closedir(dir);
}

Mat DoorNumber::getHogMat(string path)
{

  vector<float> descriptors;
  Mat trainData = imread(path.c_str(), CV_8UC1);

  HOGDescriptor *hog = new HOGDescriptor(Size(64, 96), Size(16, 16), Size(8, 8), Size(8, 8), 9);
  hog->compute(trainData, descriptors);

  int d_size = (int)descriptors.size();
  Mat hogMat(1, d_size, CV_32F);
  Mat test(trainData.size(), trainData.type());

  for (int index = 0; index < d_size; ++index)
    hogMat.ptr<float>(0)[index] = descriptors[index];

  return hogMat;
}

Mat DoorNumber::gotHogMat(Mat src)
{
  vector<float> descriptors;
  HOGDescriptor *hog = new HOGDescriptor(Size(64, 96), Size(16, 16), Size(8, 8), Size(8, 8), 9);
  hog->compute(src, descriptors);

  int d_size = (int)descriptors.size();
  Mat hogMat(1, d_size, CV_32F);

  for (int index = 0; index < d_size; ++index)
    hogMat.ptr<float>(0)[index] = descriptors[index];
  return hogMat;
}

void DoorNumber::KNN_train()
{
    //Train data!
    vector<string> samplePath;
    vector<int> labels;
    Door_recognition::DoorNumber::samplePath(samplePath, labels);
    //导入样本，并做好标签图
    for (int i = 0, _size = (int)samplePath.size(); i < _size; ++i)
    {
      Mat tmp = Door_recognition::DoorNumber::getHogMat(samplePath[i]);
      trainImage.push_back(tmp);
      labelImage.push_back(labels[i]);
    }
    //创建KNN，并且设置N值为5
    knn = KNearest::create();
    knn->setDefaultK(5);
    knn->setIsClassifier(true);
    //生成训练数据
    Ptr<TrainData> tData = TrainData::create(trainImage, ROW_SAMPLE, labelImage);
    cout << "It's training!" << endl;
    knn->train(tData);
}

int DoorNumber::predict(Mat inputImage)
{
    //预测函数
    if (trainImage.size == 0)
    {
      cout << "请先初始化" << endl;
      return -1;
    }

    Mat input = Door_recognition::DoorNumber::gotHogMat(inputImage);
    return (int)knn->predict(input); //返回预测结果
}

Mat DoorNumber::deal_camera(Mat srcImage)
{
    Mat grayImage, Image;
    Mat gray_ROI,src_ROI;
    cvtColor(srcImage, grayImage, COLOR_BGR2GRAY);
    threshold(grayImage, Image, 0, 255, CV_THRESH_OTSU);

    findContours(Image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    if(contours.size()!=0)
    {
      double mymax = 0;
      int max_id = 0;
      for (int i = 0; i != contours.size(); i++){
        double area = cv::contourArea(contours[i]);
        double area_rect = minAreaRect(contours[i]).boundingRect().area();
        double tmp = area / area_rect;
        if (mymax < tmp){
          mymax = tmp;
          max_id = i;
        }
      }
      std::vector<cv::Point2f> poly;
      cv::approxPolyDP(contours[max_id], poly, 30, true); // 多边形逼近，精度(即最小边长)设为30是为了得到4个角点
      cv::Point2f pts_src[] = { poly[0],poly[3],poly[2],poly[1]};

      int line1 = sqrt((poly[1].y - poly[0].y)*(poly[1].y - poly[0].y) + (poly[1].x - poly[0].x)*(poly[1].x - poly[0].x));
      int line2 = sqrt((poly[3].y - poly[0].y)*(poly[3].y - poly[0].y) + (poly[3].x - poly[0].x)*(poly[3].x - poly[0].x));

      if (line1 > line2)
      {
        //cout<<"till to left!"<<endl;
        pts_src[0] =poly[1];
        pts_src[1] =poly[0];
        pts_src[2] =poly[3];
        pts_src[3] =poly[2];
      }
      cv::Rect roi_rect = minAreaRect(contours[max_id]).boundingRect();
      //cout<<"grayimgae size:"<<"rows"<<grayImage.rows<<"cols"<<grayImage.cols;
      //cout<<"roi_rect position:"<<"tl x:"<<roi_rect.tl().x<<"tl y:"<<roi_rect.tl().y<<"br x:"<<roi_rect.br().x<<"br y:"<<roi_rect.br().y<<endl;
      /*if (roi_rect.tl().x < 0 ||roi_rect.tl().y<0 )
         return srcImage;
      if (roi_rect.br().x > 640 || roi_rect.br().y > 480)
          return srcImage;
    try{
        grayImage = grayImage(roi_rect);
    }
    catch(...){
      cout<<"Wrong roi!"<<endl;
      return srcImage;
    }*/

      src_ROI = perspectivetransfer(srcImage,max_id);
      //以下为对ROI区域进行二值化，并进行识别程序
      cvtColor(src_ROI, gray_ROI, COLOR_BGR2GRAY);
      threshold(gray_ROI, Image, 0, 255, CV_THRESH_OTSU | CV_THRESH_BINARY_INV);

      int size = 1;//much better than original 5
      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(size, size));
      cv::erode(Image, Image, kernel);
      cv::dilate(Image, Image, kernel);
      vector<vector<Point> > contours_2;
      findContours(Image, contours_2, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
      if (contours_2.size()>=3)
      {
        std::vector<cv::Rect> vec;
        for (int i = 0; i != contours_2.size(); i++){
          cv::Rect roi = boundingRect(contours_2[i]);

          if (roi.width < Image.cols / 6.0 || roi.width >Image.cols / 3.0)
            continue;
          if (roi.height > 0.8 * Image.rows || roi.height < 0.3 * Image.rows)
            continue;
          vec.push_back(roi);
        }
        if (vec.size() == 3){
          sort(vec.begin(), vec.end(), [](const cv::Rect & a, const cv::Rect & b){return a.x < b.x; });
          vector<Mat> nums;//number pic
          vector<int> num;//int number
          for (auto item : vec){
            nums.push_back(Image(item));}
          for (auto item : nums){
            Mat pre;
            resize(item, pre, Size(64,96));
            result = predict(pre);
            num.push_back(result);
          }
          std::string res;
          std::for_each(num.begin(), num.end(), [&](int data){ res += std::to_string(data); });
          //Point2f pts_src = computecornerpoint(max_id);
          int abs_03 =abs(pts_src[0].x - pts_src[3].x);
          int abs_12 =abs(pts_src[1].x - pts_src[2].x);
          //int abs_01 =abs(pts_src[0].y - pts_src[1].y);
          //int abs_23 =abs(pts_src[1].x - pts_src[2].x);
          int d_21 = pts_src[2].y - pts_src[1].y;
          int d_30 = pts_src[3].y - pts_src[0].y;
          float r1 = computedistance(d_21);
          float r2 = computedistance(d_30);
          float dis = (r1+r2)/2;
          if(judge_door_number_center(dis,pts_src[0],pts_src[2])){
            if (abs_03< 1 && abs_12 <1 && res == "446" ){
            //cout<<"Rectangle Width:"<<roi_rect.width<<"Height:"<<roi_rect.height<<endl;
            //cout<<"r1:"<<r1<<"r2:"<<r2<<endl;
            Point2f point_r = computecrosspoint(r1,r2);
            //cout<<"Camera position point :"<<point_r<<endl;
            }
          }
          rectangle(srcImage,roi_rect, Scalar(0, 0, 255), 3);
          positiosn = Point(roi_rect.br().x - 5, roi_rect.br().y + 20);
          putText(srcImage,res,positiosn,1, 3.0,Scalar(0, 255, 255),2);//在屏幕上打印字
          std_msgs::String door_msg;
          std::stringstream ss;
          ss << res;
          cout<<"Door number:"<<res<<endl;
          door_msg.data = ss.str();
          result_pub.publish(door_msg);
         // cout<<"Publish recognization result:"<<res<<endl;
        }
      }
    }
    return srcImage;
}

Mat DoorNumber::perspectivetransfer(Mat srcImage,int max_id){
  //获取门牌号角点,and perspective transform to 180,110 size picture
  std::vector<cv::Point2f> poly;
  cv::approxPolyDP(contours[max_id], poly, 30, true); // 多边形逼近，精度(即最小边长)设为30是为了得到4个角点
  cv::Point2f pts_src[] = { poly[0],poly[3],poly[2],poly[1]};
  //cout<<" p1:"<<poly[0].x<<poly[0].y<<endl;
  //cout<<" p2:"<<poly[1].x<<poly[1].y<<endl;
  //cout<<" p3:"<<poly[2].x<<poly[2].y<<endl;
  //cout<<" p4:"<<poly[3].x<<poly[3].y<<endl;

  int line1 = sqrt((poly[1].y - poly[0].y)*(poly[1].y - poly[0].y) + (poly[1].x - poly[0].x)*(poly[1].x - poly[0].x));
  int line2 = sqrt((poly[3].y - poly[0].y)*(poly[3].y - poly[0].y) + (poly[3].x - poly[0].x)*(poly[3].x - poly[0].x));

  if (line1 > line2)
  {
    //cout<<"till to left!"<<endl;
    pts_src[0] =poly[1];
    pts_src[1] =poly[0];
    pts_src[2] =poly[3];
    pts_src[3] =poly[2];//{ poly[1],poly[0],poly[3],poly[2]};
  }
  //Point2f pts_src[]={};
  //computecornerpoint(max_id,pts_src);
  //cout<<"pts_src:"<<pts_src[0]<<pts_src[1]<<endl;
  //透视变换后的角点
  Point2f pts_dst[] = {Point(0, 0),Point(180, 0),
  Point(180, 110) ,Point(0, 110) };
  Mat &&M = cv::getPerspectiveTransform(pts_src, pts_dst);
  Mat warp;
  Mat PerspectivedImg(110, 180, CV_8UC1);
  PerspectivedImg.setTo(0);
  cv::warpPerspective(srcImage, warp, M, PerspectivedImg.size(), cv::INTER_LINEAR , cv::BORDER_REPLICATE);
  //imshow("After Perspectived", warp);
  referenceline(srcImage,pts_src);//add reference lines
  return warp;
}

void DoorNumber::referenceline(Mat srcImage,Point2f corner_point[]){
  //cout<<"corner_point[]:"<<corner_point<<endl;
  for (int j = 0; j < 4; j++)
    {
      Point pt1,pt2,pt3,pt4;
      pt1.x = corner_point[j].x;
      pt1.y = 0;
      pt2.x = corner_point[j].x;
      pt2.y = srcImage.rows;
      pt3.x = 0;
      pt3.y = corner_point[j].y;
      pt4.x = srcImage.cols;
      pt4.y = corner_point[j].y;
      if (j % 2 == 0){
        line(srcImage, pt1, pt2, cv::Scalar(0,255,0), 1, 8);
        line(srcImage, pt3, pt4, cv::Scalar(0,255,0), 1, 8);
      }
      else{
        line(srcImage, pt1, pt2, cv::Scalar(255,0,0), 1, 8);
        line(srcImage, pt3, pt4, cv::Scalar(255,0,0), 1, 8);
      }
    }
}

float DoorNumber::computedistance(int height)
{
  float a,b,c,distance;
  a = 2497;
  b = -0.934;
  c = -1.555;
  distance = a*pow(height,b) + c;
  return distance;
}

Point2f DoorNumber::computecrosspoint(float r1,float r2){
  Point2f p_r;
  p_r.y = (81+r2*r2-r1*r1)/18;
  p_r.x =sqrt(r2*r2-(p_r.y)*(p_r.y));
  return p_r;
}

bool DoorNumber::judge_door_number_center(float dis, Point2f p_0, Point2f p_2)
{
  geometry_msgs::Twist msg;
          msg.linear.x = 0.0;
          msg.angular.z = 0.0;
  float center =  (p_2.x +  p_0.x)/2;
  //判断门牌号中心是否在特定范围
  if(abs(center-320) < 5){
  msg.angular.z = 0;
  cmd_pub.publish(msg);
  //cout<<"STOP!!!!"<<endl;
  return true;
  }
  //如未在,则设定机器人根据角度偏差旋转一定时间
  else{
    float side = (5.3/(p_2.y - p_0.y))*abs(center-320);
    if (side > dis)
      return false;
    float theta = asin(side/dis);
    //cout<<"相差"<<theta<<"弧度"<<endl;
    float t = theta/0.5;
    ///cout<<"需要用时t is:"<<t<<endl;
    if(center-320 < 0){
      msg.angular.z = 0.5;
      //cout<<"向左转"<<theta*57.3<<"度"<<endl;
    }
    else{
      msg.angular.z = -0.5;
      //cout<<"向右转"<<theta*57.3<<"度"<<endl;
    }
    cmd_pub.publish(msg);
    sleep(t);
    msg.angular.z = 0.0;
    cmd_pub.publish(msg);
  }
}

}  // namespace Door_recognition
