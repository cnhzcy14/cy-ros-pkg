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

#include "gige/gige.h"
#include <boost/thread.hpp>


namespace gige
{

GigeCamera::GigeCamera():
    pipeline_(&stream_)
{
    PvString address("192.168.1.3");
    PvResult result_;	
    //Find all GEV Devices on the network.
    system_.SetDetectionTimeout( 2000 );
    result_ = system_.Find();
    if(!result_.IsOK())
    {
        printf( "PvSystem::Find Error: %s", result_.GetCodeString().GetAscii() );
    }
    

    //Get the number of GEV Interfaces that were found using GetInterfaceCount.
    PvUInt32 interface_count_ = system_.GetInterfaceCount();

    //Display information about all found interface
    //For each interface, display information about all devices.
    for( PvUInt32 x = 0; x < interface_count_; x++ )
    {
        //get pointer to each of interface
        PvInterface * interface_ = system_.GetInterface( x );
        printf( "Interface %i\nMAC Address: %s\nIP Address: %s\nSubnet Mask: %s\n\n",
	        x,
	        interface_->GetMACAddress().GetAscii(),
	        interface_->GetIPAddress().GetAscii(),
	        interface_->GetSubnetMask().GetAscii() );

        //Get the number of GEV devices that were found using GetDeviceCount.
	    PvUInt32 device_count_ = interface_->GetDeviceCount();

	    for(PvUInt32 y = 0; y < device_count_ ; y++)
	    {
	        device_info_ = interface_->GetDeviceInfo( y );
	        printf( "Device %i\nMAC Address: %s\nIP Address: %s\nSerial number: %s\n\n",
	            y,
		        device_info_->GetMACAddress().GetAscii(),
		        device_info_->GetIPAddress().GetAscii(),
		        device_info_->GetSerialNumber().GetAscii() );
            if (address == device_info_->GetIPAddress()) break;
            else printf("Cannot find %s!\n", address.GetAscii());
	    }
    }
    // Get device parameters need to control streaming
    device_params_ = device_.GetGenParameters();

    // Negotiate streaming packet size
    device_.NegotiatePacketSize();
}

GigeCamera::~GigeCamera()
{
}

void GigeCamera::open()
{
    //Connect to the  GEV Device found.
    PvResult result_;
    if( device_info_ != NULL )
    {
	    printf("Connecting to %s\n",device_info_->GetMACAddress().GetAscii());

	    result_ = device_.Connect( device_info_ );
	    if(!result_.IsOK())
        {
	        printf("Unable to connect to %s\n", device_info_->GetMACAddress().GetAscii());
	    }
	    else
	    {
	        printf("Successfully connected to %s\n", device_info_->GetMACAddress().GetAscii());
	    }
    }
    else
    {
	    printf( "No device found\n" );
    }
    //return 0;
} 


void GigeCamera::close()
{

    printf( "Disconnecting device %s \n", device_info_->GetIPAddress().GetAscii());
    device_.Disconnect();
}


void GigeCamera::start()
{
    // Open stream - have the PvDevice do it for us
    printf( "Opening stream to device\n" );
    stream_.Open( device_info_->GetIPAddress() );

    
    // Reading payload size from device
    PvInt64 size_ = 0;
    device_params_->GetIntegerValue( "PayloadSize", size_ );

    // Set the Buffer size and the Buffer count
    pipeline_.SetBufferSize( static_cast<PvUInt32>( size_ ) );
    pipeline_.SetBufferCount( 16 ); // Increase for high frame rate without missing block IDs

    // Have to set the Device IP destination to the Stream
    device_.SetStreamDestination( stream_.GetLocalIPAddress(), stream_.GetLocalPort() );

    // IMPORTANT: the pipeline needs to be "armed", or started before 
    // we instruct the device to send us images
    printf( "Starting pipeline\n" );
    pipeline_.Start();

    // Get stream parameters/stats
    stream_params_ = stream_.GetParameters();

    // TLParamsLocked is optional but when present, it MUST be set to 1
    // before sending the AcquisitionStart command
    device_params_->SetIntegerValue( "TLParamsLocked", 1 );

    printf( "Resetting timestamp counter...\n" );
    device_params_->ExecuteCommand( "GevTimestampControlReset" );


    // The pipeline is already "armed", we just have to tell the device
    // to start sending us images
    printf( "Sending StartAcquisition command to device\n" );
    device_params_->ExecuteCommand( "AcquisitionStart" );

    image_thread_.reset(new boost::thread (boost::bind (&GigeCamera::imageThread, this)));

}


void GigeCamera::imageThread()
{
    PvInt64 width_, height_ = 0;
    device_params_->GetIntegerValue( "Width", width_ );
    
    device_params_->GetIntegerValue( "Height", height_ );

    cv::Mat raw_image_(cv::Size(width_,height_),CV_16U);
    cv::namedWindow("flir",cv::WINDOW_AUTOSIZE);

    char doodle_[] = "|\\-|-/";
    int doodle_index_ = 0;
    PvInt64 image_count_val_ = 0;
    double frame_rate_val_ = 0.0;
    double bandwidth_val_ = 0.0;

    // Acquire images until the user instructs us to stop
    printf( "\n<press the enter key to stop streaming>\n" );
    while ( getchar() )
    {
        // Retrieve next buffer		
        PvBuffer *buffer_ = NULL;
        PvImage *image_ = NULL;
        PvResult  operation_result_;
        PvResult result_ = pipeline_.RetrieveNextBuffer( &buffer_, 1000, &operation_result_ );
        
        if ( result_.IsOK() )
        {
            if ( operation_result_.IsOK() )
            {
	        stream_params_->GetIntegerValue( "ImagesCount", image_count_val_ );
		stream_params_->GetFloatValue( "AcquisitionRateAverage", frame_rate_val_ );
		stream_params_->GetFloatValue( "BandwidthAverage", bandwidth_val_ );
            
		// If the buffer contains an image, display width and height
		//PvUInt32 width_ = 0, height_ = 0;
		if ( buffer_->GetPayloadType() == PvPayloadTypeImage )
		{
		    // Get image specific buffer interface
		    image_ = buffer_->GetImage();

		    // Read width, height
		    //width_ = image_->GetWidth();
		    //height_ = image_->GetHeight();
		}

		printf( "%c Timestamp: %016llX BlockID: %04X W: %i H: %i %.01f FPS %.01f Mb/s\r", 
                    doodle_[ doodle_index_ ],
                    buffer_->GetTimestamp(),
                    buffer_->GetBlockID(),
		    width_,
		    height_,
                    frame_rate_val_,
                    bandwidth_val_ / 1000000.0 ); 

            }
            // We have an image - do some processing (...) and VERY IMPORTANT,
            image_->Attach(raw_image_.data,image_->GetWidth(),image_->GetHeight(),PvPixelMono16);
            cv::imshow("flir", raw_image_);
            if(cv::waitKey(30) >= 0) break;
            useImage(raw_image_);
            // release the buffer back to the pipeline
            pipeline_.ReleaseBuffer( buffer_ );
        }
        else
        {
            // Timeout
            printf( "%c Timeout\r", doodle_[ doodle_index_ ] );
        }
        ++doodle_index_ %= 6;
    }

}


void GigeCamera::stop()
{
	getchar(); // Flush key buffer for next stop
    printf( "\n\n" );

    image_thread_.reset();

    // Tell the device to stop sending images
    printf( "Sending AcquisitionStop command to the device\n" );
    device_params_->ExecuteCommand( "AcquisitionStop" );

    // If present reset TLParamsLocked to 0. Must be done AFTER the 
    // streaming has been stopped
    device_params_->SetIntegerValue( "TLParamsLocked", 0 );

    // We stop the pipeline - letting the object lapse out of 
    // scope would have had the destructor do the same, but we do it anyway
    printf( "Stop pipeline\n" );
    pipeline_.Stop();

    // Now close the stream. Also optionnal but nice to have
    printf( "Closing stream\n" );
    stream_.Close();

}

void GigeCamera::autoFocus()
{
    device_params_->ExecuteCommand( "AutoFocus" );
}

void GigeCamera::setIRFormat(int i)
{
    device_params_->SetEnumValue( "IRFormat", i );
}

void GigeCamera::setSize(int w, int h)
{
    device_params_->SetIntegerValue( "Width", w );
    device_params_->SetIntegerValue( "Height", h );
}

} // namespace gige

