#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/image_encodings.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "dynamic_reconfigure/server.h"
#include "cv_06/dynamicConfig.h"
#include "cv_06/main_class.h"
#include "pluginlib/class_list_macros.h"


int threshold_maxval=255;
int dilate_iterations=1;
int erode_iterations=1;
int medianBlur_Ksize=3;
int light_contour_area_assignment=10;
int contour_angle_assignment=8;
double ratio_of_length_difference=0.5;
double ratio_of_wideth_difference=0.5;
double more_length_width_ratio=5.0;
double less_length_width_ratio=1.0;
double less_ratio_of_x_difference=0.5;
double more_ratio_of_y_difference=2.0;

void dynamic_callback(cv_06::dynamicConfig &config)
{
    try {
        threshold_maxval = config.threshold_maxval;
        dilate_iterations = config.dilate_iterations;
        erode_iterations  = config.erode_iterations;
        light_contour_area_assignment = config.light_contour_area_assignment;
        contour_angle_assignment = config.contour_angle_assignment;
        ratio_of_length_difference = config.ratio_of_length_difference;
        ratio_of_wideth_difference = config.ratio_of_wideth_difference;
        more_length_width_ratio = config.more_length_width_ratio;
        less_length_width_ratio = config.less_length_width_ratio;
        less_ratio_of_x_difference = config.less_ratio_of_x_difference;
        more_ratio_of_y_difference = config.more_ratio_of_y_difference;
        ROS_INFO("Changes have been seted");
    }
    catch (...)
    {
        ROS_INFO("Something Went Wrong");
    }
}

namespace cv_06 {
    void MainClass::onInit() {
        ros::NodeHandle nh("~");
        image_transport::ImageTransport it_(nh);
        nh.setParam("image_transport", "compressed");
        cam_sub_ = it_.subscribeCamera("/galaxy_camera/image_raw", 10, &MainClass::onFrameCb,this); //订阅rosbag的信息
        cam_pub_ = it_.advertise("final_image/image", 1); //发布图片信息给rqt
        dynamic_reconfigure::Server<cv_06::dynamicConfig> server;
        dynamic_reconfigure::Server<cv_06::dynamicConfig>::CallbackType callback;
        callback = boost::bind(&dynamic_callback, _1);
        server.setCallback(callback);
        ros::spin();
//        ros::Rate r(10.0);
//        while (ros::ok())
//        {
//            ros::spinOnce();
//            r.sleep();
//        }
    }

    cv::RotatedRect MainClass::adjustRec(cv::RotatedRect& rec)//矫正灯条
    {

        float& width = rec.size.width;
        float& height = rec.size.height;
        float& angle = rec.angle;


        while (angle >= 90.0) angle -= 180.0;
        while (angle < -90.0) angle += 180.0;



        if (angle >= 45.0)
        {
            std::swap(width, height);
            angle -= 90.0;
        }
        else if (angle < -45.0)
        {
            std::swap(width, height);
            angle += 90.0;
        }

        return rec;
    }
    double MainClass::max(double a,double b) {
        if (a > b) return a;
        else return b;
    } //获取最大值
    void MainClass::onFrameCb(const sensor_msgs::ImageConstPtr &img, const sensor_msgs::CameraInfoConstPtr &info) {
        cv_image_ = cv_bridge::toCvCopy(img, "bgr8");
        cv::resize(cv_image_->image, cv_image_->image, cv::Size(400, 400));
        cv::Mat_<uchar> gray_pic;
        cv::cvtColor(cv_image_->image, gray_pic, cv::COLOR_BGR2GRAY);
        cv::Mat_<uchar> binary_pic;
        cv::threshold(gray_pic, binary_pic, -1, threshold_maxval, cv::THRESH_OTSU);
        cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        cv::dilate(binary_pic, binary_pic, element, cv::Point(-1, -1), dilate_iterations);
        cv::erode(binary_pic, binary_pic, element, cv::Point(-1, -1), erode_iterations);
        cv::medianBlur(binary_pic, binary_pic, medianBlur_Ksize);
        //以上 进行阈值，高帽，中值滤波处理 目的为使得灯条特征更加明显

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(binary_pic, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        // 以上获取轮廓 以下进行轮廓筛选
        std::vector<cv::RotatedRect> light_infos; //用于收纳筛选后的灯条的轮廓
        for (int i = 0; i < contours.size(); i++) {
            double light_contour_area = cv::contourArea(contours[i]);  //获取轮廓面积
            if (contours[i].size() <= 5 ||
                light_contour_area < light_contour_area_assignment) //轮廓太少的噪声不要 面积太小的不要
                continue;

            //椭圆拟合生成相应的旋转矩形  （用矩形去拟合灯条，方便获取灯条的长宽比等）
            cv::RotatedRect light_rec = fitEllipse(contours[i]);
            //矫正灯条
            adjustRec(light_rec);

            // 注释以下筛选代码的原因是可以在该rosbag中获得更好的装甲板捕捉效果 以下代码可以用于其他更难以区分装甲板的情况
//            //宽高比、凸度筛选灯条
//            if ((light_rec.size.width / light_rec.size.height) > 0.8)
//                continue;
//
//            float x = light_rec.center.x - light_rec.size.width;
//            if (x < 0)
//                continue;

//            float y = light_rec.center.y - light_rec.size.height;
//            if (y < 0)
//                continue;

//            if (light_rec.size.width / light_rec.size.height > 1.0 ||
//                light_contour_area / light_rec.size.area() < 0.5) //宽高闭 凹度
//                continue;

            //对灯条范围适当扩大
            light_rec.size.width *= 1.1;
            light_rec.size.height *= 1.1;

            light_infos.push_back(light_rec);
        }

        for (int i = 0; i < light_infos.size(); i++) {
            for (int j = i + 1; j < light_infos.size(); j++) {
                //以下判断是否为相同灯条
                double contour_angle = abs(light_infos[i].angle - light_infos[j].angle); //角度差
                if (contour_angle >= contour_angle_assignment)
                    continue;

                //长度差比率
                double contour_len1 = abs(light_infos[i].size.height - light_infos[j].size.height) /
                                      max(light_infos[i].size.height, light_infos[j].size.height);

                //宽度差比率
                double contour_len2 = abs(light_infos[i].size.width - light_infos[j].size.width) /
                                      max(light_infos[i].size.width, light_infos[j].size.width);

                if (contour_len1 > ratio_of_length_difference || contour_len2 > ratio_of_wideth_difference)
                    continue;
                double nh, nw, yDiff, xDiff;
                nh = (light_infos[i].size.height + light_infos[j].size.height) / 2; //高度平均值
                // 宽度平均值
                nw = sqrt((light_infos[i].center.x - light_infos[j].center.x) *
                          (light_infos[i].center.x - light_infos[j].center.x) +
                          (light_infos[i].center.y - light_infos[j].center.y) *
                          (light_infos[i].center.y - light_infos[j].center.y));

                double ratio = nw / nh; //匹配到的装甲板的长宽比
                xDiff = abs(light_infos[i].center.x - light_infos[j].center.x) / nh; //x差比率
                yDiff = abs(light_infos[i].center.y - light_infos[j].center.y) / nh; //y差比率
                if (ratio < less_length_width_ratio || ratio > more_length_width_ratio ||
                    xDiff < less_ratio_of_x_difference || yDiff > more_ratio_of_y_difference)
                    continue;

                //以上筛选相同灯条的拟合矩形完毕

                cv::RotatedRect armor; //创建捕获装甲板的矩形
                armor.center.x = (light_infos[i].center.x + light_infos[j].center.x) / 2.; //x坐标
                armor.center.y = (light_infos[i].center.y + light_infos[j].center.y) / 2.; //y坐标
                armor.angle = (light_infos[i].angle + light_infos[j].angle) / 2.; //角度
                armor.size.height = nh;
                armor.size.width = nw;
                cv::Point2f *touchVertices = new cv::Point2f[4]; //容纳装甲板四顶点的容器
                armor.points(touchVertices);
                std::vector<cv::Point2f> corner_point_saver; //给装甲板四顶点的vector容器 方便后续操作

                //对装甲板四顶点进行画线
                for (int i = 0; i < 4; ++i) {
                    cv::line(cv_image_->image, touchVertices[i % 4], touchVertices[(i + 1) % 4], cv::Scalar(255), 2);
                    corner_point_saver.push_back(touchVertices[i]);
                }

                //获取位姿变换
                std::vector<cv::Point3f> corner_point_saver_for_real; // 容纳世界坐标系的vector容器
                //以下为在世界坐标系下定义的装甲板四顶点的坐标
                corner_point_saver_for_real.push_back(cv::Point3f(-117.5, 30.5, 0));
                corner_point_saver_for_real.push_back(cv::Point3f(-117.5, -30.5, 0));
                corner_point_saver_for_real.push_back(cv::Point3f(117.5, -30.5, 0));
                corner_point_saver_for_real.push_back(cv::Point3f(117.5, 30.5, 0));
                cv::Mat rvec = cv::Mat_<double>(3, 1);
                cv::Mat tvec = cv::Mat_<double>(3, 1);
                cv::Mat K_Mat = (cv::Mat_<double>(3, 3)
                        << info->K[0], info->K[1], info->K[2], info->K[3], info->K[4], info->K[5], info->K[6], info->K[7], info->K[8]);
                cv::Mat D_Mat = (cv::Mat_<double>(1, 5) << info->D[0], info->D[1], info->D[2], info->D[3], info->D[4]);
                cv::solvePnP(corner_point_saver_for_real, corner_point_saver, K_Mat, D_Mat, rvec, tvec, false);
                //以上获取到了位姿变换的旋转向量和平移向量

                //在装甲板上绘制坐标系
                std::vector<cv::Point3f> new_vec;
                new_vec.push_back(cv::Point3f(0, 0, 0));
                new_vec.push_back(cv::Point3f(100, 0, 0));
                new_vec.push_back(cv::Point3f(0, 100, 0));
                new_vec.push_back(cv::Point3f(0, 0, 100));

                std::vector<cv::Point2f> imagePoints;
                cv::projectPoints(new_vec, rvec, tvec, K_Mat, D_Mat, imagePoints);//根据三维世界坐标获取二维图像坐标点

                cv::Point2f armor_center(armor.center.x, armor.center.y);

                cv::line(cv_image_->image, armor_center, imagePoints[1], cv::Scalar(0, 255, 255), 2);
                cv::line(cv_image_->image, armor_center, imagePoints[2], cv::Scalar(0, 255, 0), 2);
                cv::line(cv_image_->image, armor_center, imagePoints[3], cv::Scalar(0, 0, 255), 2);

                // 在tf中显示装甲板坐标系
                cv::Mat r_mat = cv::Mat_<double>(3, 3);
                cv::Rodrigues(rvec, r_mat);//将旋转向量转化为旋转矩阵
                tf::Matrix3x3 t_m(r_mat.at<double>(0, 0), r_mat.at<double>(0, 1), r_mat.at<double>(0, 2),
                                  r_mat.at<double>(1, 0), r_mat.at<double>(1, 1), r_mat.at<double>(1, 2),
                                  r_mat.at<double>(2, 0), r_mat.at<double>(2, 1), r_mat.at<double>(2, 2));
                tf::Vector3 t_tvec(0, 0, 0.5); //平移向量 定义在tf中坐标系的原点位置

                //以下为获取转化必要的参数
                tf::Quaternion quaternion; //定义四元数
                double r;
                double p;
                double y;
                t_m.getRPY(r, p, y);
                quaternion.setRPY(r, p, y);
                tf::Transform transform;
                transform.setRotation(quaternion);
                transform.setOrigin(t_tvec);
                tf::StampedTransform stamped_Transfor(transform, ros::Time::now(), "camera_optional_frame",
                                                      "target_ID:1");
                //获取完毕 以下创建tf广播并且发送信息给tf
                static tf::TransformBroadcaster broadcaster;
                broadcaster.sendTransform(stamped_Transfor);
            }
            //定义发给rqt的图片信息类型 将图片的opencv格式转化为ROS格式
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image_->image).toImageMsg();
            cam_pub_.publish(msg);

        }
    }


}
PLUGINLIB_EXPORT_CLASS(cv_06::MainClass, nodelet::Nodelet);


