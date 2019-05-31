#include "doornumber.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include "sensor_msgs/image_encodings.h"
#include <std_msgs/String.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace enc = sensor_msgs::image_encodings;
namespace Door_recognition {

DoorNumber::DoorNumber():it_(nh_)
{
    image_pub_ = it_.advertise("camera/process", 1);
    result_pub = nh_.advertise<std_msgs::String>("door_result",1);
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
      cv::Rect roi_rect = minAreaRect(contours[max_id]).boundingRect();
      //cout<<"grayimgae size:"<<"rows"<<grayImage.rows<<"cols"<<grayImage.cols;
      //cout<<"roi_rect position:"<<"tl x:"<<roi_rect.tl().x<<"tl y:"<<roi_rect.tl().y<<"br x:"<<roi_rect.br().x<<"br y:"<<roi_rect.br().y<<endl;
      if (roi_rect.tl().x < 0 ||roi_rect.tl().y<0 )
         return srcImage;
      if (roi_rect.br().x > 640 || roi_rect.br().y > 480)
          return srcImage;
    try{
        grayImage = grayImage(roi_rect);
    }
    catch(...){
      cout<<"Wrong roi!"<<endl;
      return srcImage;
    }
      threshold(grayImage, Image, 0, 255, CV_THRESH_OTSU | CV_THRESH_BINARY_INV);

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

          if (roi.width < Image.cols / 8.0 || roi.width >Image.cols / 3.0)
            continue;
          if (roi.height > 0.6 * Image.rows || roi.height < 0.2 * Image.rows)
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
          rectangle(srcImage,roi_rect, Scalar(0, 0, 255), 3);
          positiosn = Point(roi_rect.br().x - 5, roi_rect.br().y + 20);
          putText(srcImage,res,positiosn,1, 3.0,Scalar(0, 255, 255),2);//在屏幕上打印字
          std_msgs::String door_msg;
          std::stringstream ss;
          ss << res;
          door_msg.data = ss.str();
          result_pub.publish(door_msg);
          cout<<"Publish recognization result:"<<res<<endl;
        }
      }
    }
    return srcImage;
}

}  // namespace Door_recognition
