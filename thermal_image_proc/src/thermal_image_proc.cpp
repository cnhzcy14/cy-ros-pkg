/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2012, Ye Cheng
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/


#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <dynamic_reconfigure/server.h>
//#include <thermal_image_proc/FlirConfig.h>


namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class ImageConverter
{
    ros::NodeHandle nh_;
    ros::NodeHandle nh_temp_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    //ros::Publisher temp_pub_;

public:
    int interest_y;
    int interest_x;

    ImageConverter():   
        it_(nh_)
    {
        image_pub_ = it_.advertise("out", 1);
        image_sub_ = it_.subscribe("image_raw", 1, &ImageConverter::imageCb, this);

        //temp_pub_ = nh_temp_.advertise<std_msgs::Float32>("temperature", 1000);

        cv::namedWindow(WINDOW);
    }

    ~ImageConverter()
    {
        cv::destroyWindow(WINDOW);
    }

//void configCb(learning_image_transport::FlirConfig &config, uint32_t level)
//{
//  interest_y = config.row;
//  interest_x = config.col;
//} // end configCallback()

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        std_msgs::Float32 temp_msg_;
        unsigned short minpix = 0xFFFF;
        unsigned short maxpix = 0;
        cv::Mat gray_img_(cv::Size(msg->width,msg->height),CV_8U);
        cv_bridge::CvImagePtr cv_ptr_;
        try
        {
            cv_ptr_ = cv_bridge::toCvCopy(msg, enc::MONO16);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        unsigned short pixel_val_;
        pixel_val_ = cv_ptr_->image.at<unsigned short>(interest_y,interest_x);
        temp_msg_.data = (float)(pixel_val_ / 10.0f) - 273.15f;

        
        for( int y = 0; y < cv_ptr_->image.rows; y++ )
        { 
            for( int x = 0; x < cv_ptr_->image.cols ; x++ )
            { 
                if (cv_ptr_->image.at<unsigned short>(y,x) < minpix) minpix = cv_ptr_->image.at<unsigned short>(y,x);
                if (cv_ptr_->image.at<unsigned short>(y,x) > maxpix) maxpix = cv_ptr_->image.at<unsigned short>(y,x);
            }
        }
    
        float span = (float)(maxpix - minpix + 1);
        for( int y = 0; y < cv_ptr_->image.rows; y++ )
        { 
            for( int x = 0; x < cv_ptr_->image.cols ; x++ )
            { 
                gray_img_.at<uchar>(y,x) = cv::saturate_cast<uchar>(((cv_ptr_->image.at<unsigned short>(y,x) - minpix) / span) * 0xFF);

            }
        }
        cv_ptr_->image = gray_img_;
        cv_ptr_->encoding = "bgr8";
        cv::applyColorMap(cv_ptr_->image, cv_ptr_->image, cv::COLORMAP_JET);
        cv::circle(cv_ptr_->image, cv::Point(interest_y, interest_x), 5, CV_RGB(255,255,255));
        cv::imshow(WINDOW, cv_ptr_->image);
        cv::waitKey(3);
    
        //temp_pub_.publish(temp_msg_);
        image_pub_.publish(cv_ptr_->toImageMsg());
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;

  //dynamic_reconfigure::Server<learning_image_transport::FlirConfig> dr_srv;
  //dynamic_reconfigure::Server<learning_image_transport::FlirConfig>::CallbackType cb;
  //cb = boost::bind(&ImageConverter::configCb, &ic, _1, _2);
  //dr_srv.setCallback(cb);

    ros::spin();
    return 0;
}

