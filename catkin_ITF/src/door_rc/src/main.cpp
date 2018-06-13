#include<ros/ros.h>
#include "doornumber.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "DoorNumber_recognition");
    Door_recognition::DoorNumber dn;
    ros::spin();

    return 0;
}
