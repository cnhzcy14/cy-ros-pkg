/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012 (Simon) CHENG Ye
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

#ifdef WIN32
#include <windows.h>
#include <conio.h>
#include <process.h>
#endif // WIN32

#include <stdio.h>

#ifdef _UNIX_

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>




int _kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

#define _getch getchar

#endif // _UNIX_

#include <PvDeviceFinderWnd.h>
#include <PvDevice.h>
#include <PvPipeline.h>
#include <PvBuffer.h>
#include <PvStream.h>
#include <PvStreamRaw.h>
#include <PvDisplayWnd.h>
#include <PvFilterRGB.h>
#include <PvBufferConverter.h>
#include <PvBufferWriter.h>
#include <PvSystem.h>
#include <opencv2/opencv.hpp>
#include <boost/thread.hpp>


namespace gige
{

class GigeCamera
{
public:
    GigeCamera();
    ~GigeCamera();
    void open();
    void close();
    void start();
    void stop();
    void autoFocus();
    void setIRFormat(int);
    void setSize(int, int);
    boost::function<void(const cv::Mat &image)> useImage; 

private:
    PvSystem system_;         
    PvDevice device_;
    PvStream stream_;
    PvDeviceInfo *device_info_;
    PvPipeline pipeline_;
    PvGenParameterArray *device_params_;
    PvGenParameterArray *stream_params_;
    void imageThread();
    boost::shared_ptr<boost::thread> image_thread_;
     
};


} // namespace gige


