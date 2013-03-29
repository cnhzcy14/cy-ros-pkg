/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2012, (Simon) CHENG Ye
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

#include "gige/gige.h"
#include <stdio.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>
//#include "gige_camera/FlirConfig.h"


double pix2temp(unsigned short pValue)
{
    return (float)(pValue / 10.0f) - 273.15f;
}

class FLIRNode
{
private:
    // ROS
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher pub_;

    // ROS Message
    sensor_msgs::ImagePtr img_;

    // FLIR Camera
    boost::scoped_ptr <gige::GigeCamera> cam_;

    // Dynamic Reconfigure
    //dynamic_reconfigure::Server<gige_camera::FlirConfig> reconfig_svr_;

public:
    FLIRNode(const ros::NodeHandle& node_handle):
        nh_(node_handle),
        it_(nh_),
        pub_(it_.advertise("image_raw",1)),
        cam_(NULL)
    {
        cam_.reset(new gige::GigeCamera());
        cam_->open();
        cam_->start();
        cam_->useImage = boost::bind(&FLIRNode::publishImage, this, _1);
        //reconfig_svr_.setCallback(boost::bind(&FLIRNode::configCb, this, _1, _2));
        ROS_INFO(" GigE camera start ====== ");
    }

    ~FLIRNode()
    {
        cam_->stop();
        cam_->close();
        cam_.reset();
        ROS_INFO(" GigE camera stop ====== ");
    }

    void publishImage(const cv::Mat &image)
    {
        cv_bridge::CvImage cv_image;
        //TODO: pass the encoding type from camera driver. 
        cv_image.encoding = "mono16";
        cv_image.image = image;
        img_ = cv_image.toImageMsg();
        pub_.publish(img_);
    }

/*
    void configCb(gige_camera::FlirConfig &config, uint32_t level)
    {
        if (level >= (uint32_t)driver_base::SensorLevels::RECONFIGURE_STOP) cam_->stop();

        cam_->setIRFormat(config.IRFormat);
        if (config.AutoFocus) cam_->autoFocus();
        //cam_->setSize(config.Width, config.Height);

        if (level >= (uint32_t)driver_base::SensorLevels::RECONFIGURE_STOP) cam_->start();
    }
*/
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "flir_camera");
    ros::NodeHandle nh;
    FLIRNode fn(nh);
    ros::spin();
    return 0;
}


