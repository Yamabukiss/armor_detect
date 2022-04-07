#pragma once
#include "nodelet/nodelet.h"
namespace cv_06{
    class MainClass : public nodelet::Nodelet
    {
    public:
        virtual void onInit() ;
        virtual void onFrameCb(const sensor_msgs::ImageConstPtr &img, const sensor_msgs::CameraInfoConstPtr &info);
        virtual cv::RotatedRect adjustRec(cv::RotatedRect& rec);
        virtual double max(double a,double b);
        cv::Mat_<cv::Vec3b> image;
        image_transport::CameraSubscriber cam_sub_;
        image_transport::Publisher cam_pub_;
        cv_bridge::CvImagePtr cv_image_;
    };
}