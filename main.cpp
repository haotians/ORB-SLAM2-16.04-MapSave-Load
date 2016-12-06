#include <iostream>


//include opencv related headers
#include <stdio.h>
#include <cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cxcore.h>
#include <highgui.h>
#include <vector>

//realsense
#include <inttypes.h>
#include <librealsense/rs.hpp>

//ORB
#include<System.h>

//time
#include <sys/time.h>


using namespace std;


int display(cv::Mat im)
{
    cv::imshow("LOL", im);
    return cv::waitKey(10);
}


double EGetCurTime()
{
	double dTime=0.0;
	struct timeval tv={0};
	struct timezone tz={0};

	gettimeofday (&tv,&tz);

	dTime = tv.tv_sec+(double)tv.tv_usec/1000000;

	return dTime;
}

void Original()
{
 //init ORB
    cout<< "Init ORB-RGBD system ...\n" << endl;
    ORB_SLAM2::System SLAM("ORBvoc.txt","Realsense.yaml",ORB_SLAM2::System::RGBD);

    cout << "Init Realsense ...\n" << endl;
    //init realsense
    int nFrameHeight = 480;
    int nFrameWidth = 640;
    int nFrameRate = 30;

    // if p parameter is less than 0, then realsense is activated
    rs::log_to_console(rs::log_severity::warn);
    rs::context ctx;
    if(ctx.get_device_count() == 0) throw std::runtime_error("No device detected. Is it plugged in?");
    rs::device & dev = *ctx.get_device(0);

    //TODO: switch case for realsense format

    dev.enable_stream(rs::stream::depth, nFrameWidth, nFrameHeight, rs::format::z16, nFrameRate);
    dev.enable_stream(rs::stream::color, nFrameWidth, nFrameHeight, rs::format::rgb8, nFrameRate);
    dev.enable_stream(rs::stream::infrared, nFrameWidth, nFrameHeight, rs::format::y8, nFrameRate);
    try { dev.enable_stream(rs::stream::infrared2, 0, 0, rs::format::any, 0); } catch(...) {}

    //set control paranters for depth quality, use preset temporarily
    rs::apply_depth_control_preset(&dev,4);

    // Compute field of view for each enabled stream
    for(int i = 0; i < 4; ++i)
    {
        auto stream = rs::stream(i);
        if(!dev.is_stream_enabled(stream)) continue;
        auto intrin = dev.get_stream_intrinsics(stream);
        std::cout << "Capturing " << stream << " at " << intrin.width << " x " << intrin.height;
//        std::cout << setprecision(1) << std::fixed << ", fov = " << intrin.hfov() << " x " << intrin.vfov() << ", distortion = " << intrin.model() << std::endl;
    }
    dev.start();

    cout << "Start processing sequence ..." << endl;
    while(1)
    {
        //wait for frame
        dev.wait_for_frames();

        //make buffer
        uint8_t ColorImage[nFrameHeight][nFrameWidth][3];
        float DepthImage[nFrameHeight][nFrameWidth];

        //get data from realsense from stream
        const uint8_t * pColorImageRaw = (const uint8_t *)dev.get_frame_data(rs::stream::color);
        const uint16_t * pDepthImageRaw = (const uint16_t *)dev.get_frame_data(rs::stream::depth_aligned_to_color);

        //RGB and D data construct
        for(int dx = 0; dx < nFrameHeight; ++dx)
        {
            for(int dy = 0; dy < nFrameWidth; ++dy)
            {
                //depth info
                DepthImage[dx][dy] = (float)pDepthImageRaw[dx*nFrameWidth + dy];
                //cout<<"value: "<<pDepthImageRaw[dx*nFrameWidth + dy]/1000.0<<endl;

                //color info
                for(int dw = 0; dw < 3; ++dw)
                {
                    ColorImage[dx][dy][2-dw] = pColorImageRaw[(dx*nFrameWidth + dy)*3 + dw];
                }
            }
        }

        //to cv::Mat
        cv::Mat idepth_image(nFrameHeight, nFrameWidth, CV_32F, DepthImage);
        cv::Mat ColorImage8bit(nFrameHeight, nFrameWidth, CV_8UC3, ColorImage);

        // Pass the image to the SLAM system
       SLAM.TrackRGBD(ColorImage8bit,idepth_image,EGetCurTime());
       cv::waitKey(20);

    }

    // Stop all threads
    SLAM.Shutdown();

    return;
}

void Modified(bool bRestart)
{

    //init ORB
    cout<< "Init ORB-RGBD system ...\n" << endl;

    if(bRestart)
    {
        ORB_SLAM2::System SLAM("ORBvoc.txt","Realsense.yaml",ORB_SLAM2::System::RGBD,true,true);
         cout << "Init Realsense ...\n" << endl;
        //init realsense
        int nFrameHeight = 480;
        int nFrameWidth = 640;
        int nFrameRate = 30;

        // if p parameter is less than 0, then realsense is activated
        rs::log_to_console(rs::log_severity::warn);
        rs::context ctx;
        if(ctx.get_device_count() == 0) throw std::runtime_error("No device detected. Is it plugged in?");
        rs::device & dev = *ctx.get_device(0);

        //TODO: switch case for realsense format

        dev.enable_stream(rs::stream::depth, nFrameWidth, nFrameHeight, rs::format::z16, nFrameRate);
        dev.enable_stream(rs::stream::color, nFrameWidth, nFrameHeight, rs::format::rgb8, nFrameRate);
        dev.enable_stream(rs::stream::infrared, nFrameWidth, nFrameHeight, rs::format::y8, nFrameRate);
        try
        {
            dev.enable_stream(rs::stream::infrared2, 0, 0, rs::format::any, 0);
        }
        catch(...)
        {

        }

        //set control paranters for depth quality, use preset temporarily
        rs::apply_depth_control_preset(&dev,4);

        // Compute field of view for each enabled stream
        for(int i = 0; i < 4; ++i)
        {
            auto stream = rs::stream(i);
            if(!dev.is_stream_enabled(stream)) continue;
            auto intrin = dev.get_stream_intrinsics(stream);
            std::cout << "Capturing " << stream << " at " << intrin.width << " x " << intrin.height;
    //        std::cout << setprecision(1) << std::fixed << ", fov = " << intrin.hfov() << " x " << intrin.vfov() << ", distortion = " << intrin.model() << std::endl;
        }
        dev.start();

        cout << "Start processing sequence ..." << endl;
        while(1)
        {
            //wait for frame
            dev.wait_for_frames();

            //make buffer
            uint8_t ColorImage[nFrameHeight][nFrameWidth][3];
            float DepthImage[nFrameHeight][nFrameWidth];

            //get data from realsense from stream
            const uint8_t * pColorImageRaw = (const uint8_t *)dev.get_frame_data(rs::stream::color);
            const uint16_t * pDepthImageRaw = (const uint16_t *)dev.get_frame_data(rs::stream::depth_aligned_to_color);

            //RGB and D data construct
            for(int dx = 0; dx < nFrameHeight; ++dx)
            {
                for(int dy = 0; dy < nFrameWidth; ++dy)
                {
                    //depth info
                    DepthImage[dx][dy] = (float)pDepthImageRaw[dx*nFrameWidth + dy];
                    //cout<<"value: "<<pDepthImageRaw[dx*nFrameWidth + dy]/1000.0<<endl;

                    //color info
                    for(int dw = 0; dw < 3; ++dw)
                    {
                        ColorImage[dx][dy][2-dw] = pColorImageRaw[(dx*nFrameWidth + dy)*3 + dw];
                    }
                }
            }

            //to cv::Mat
            cv::Mat idepth_image(nFrameHeight, nFrameWidth, CV_32F, DepthImage);
            cv::Mat ColorImage8bit(nFrameHeight, nFrameWidth, CV_8UC3, ColorImage);

            // Pass the image to the SLAM system
           SLAM.TrackRGBD(ColorImage8bit,idepth_image,EGetCurTime());
           //cv::waitKey(20);

           char key = display(ColorImage8bit );
           if(key == 'q')
           {
                if(bRestart)
                {
                    SLAM.SaveMap("ORBMap.map.bin");
                }
                break;
           }

        }
        // Stop all threads
        SLAM.Shutdown();
    }
    else
    {
        ORB_SLAM2::System SLAM("ORBvoc.txt","Realsense.yaml",ORB_SLAM2::System::RGBD,true,false,"ORBMap.map.bin");
        SLAM.ActivateLocalizationMode();
         cout << "Init Realsense ...\n" << endl;
        //init realsense
        int nFrameHeight = 480;
        int nFrameWidth = 640;
        int nFrameRate = 30;

        // if p parameter is less than 0, then realsense is activated
        rs::log_to_console(rs::log_severity::warn);
        rs::context ctx;
        if(ctx.get_device_count() == 0) throw std::runtime_error("No device detected. Is it plugged in?");
        rs::device & dev = *ctx.get_device(0);

        //TODO: switch case for realsense format

        dev.enable_stream(rs::stream::depth, nFrameWidth, nFrameHeight, rs::format::z16, nFrameRate);
        dev.enable_stream(rs::stream::color, nFrameWidth, nFrameHeight, rs::format::rgb8, nFrameRate);
        dev.enable_stream(rs::stream::infrared, nFrameWidth, nFrameHeight, rs::format::y8, nFrameRate);
        try
        {
            dev.enable_stream(rs::stream::infrared2, 0, 0, rs::format::any, 0);
        }
        catch(...)
        {

        }

        //set control paranters for depth quality, use preset temporarily
        rs::apply_depth_control_preset(&dev,4);

        // Compute field of view for each enabled stream
        for(int i = 0; i < 4; ++i)
        {
            auto stream = rs::stream(i);
            if(!dev.is_stream_enabled(stream)) continue;
            auto intrin = dev.get_stream_intrinsics(stream);
            std::cout << "Capturing " << stream << " at " << intrin.width << " x " << intrin.height;
    //        std::cout << setprecision(1) << std::fixed << ", fov = " << intrin.hfov() << " x " << intrin.vfov() << ", distortion = " << intrin.model() << std::endl;
        }
        dev.start();

        cout << "Start processing sequence ..." << endl;
        while(1)
        {
            //wait for frame
            dev.wait_for_frames();

            //make buffer
            uint8_t ColorImage[nFrameHeight][nFrameWidth][3];
            float DepthImage[nFrameHeight][nFrameWidth];

            //get data from realsense from stream
            const uint8_t * pColorImageRaw = (const uint8_t *)dev.get_frame_data(rs::stream::color);
            const uint16_t * pDepthImageRaw = (const uint16_t *)dev.get_frame_data(rs::stream::depth_aligned_to_color);

            //RGB and D data construct
            for(int dx = 0; dx < nFrameHeight; ++dx)
            {
                for(int dy = 0; dy < nFrameWidth; ++dy)
                {
                    //depth info
                    DepthImage[dx][dy] = (float)pDepthImageRaw[dx*nFrameWidth + dy];
                    //cout<<"value: "<<pDepthImageRaw[dx*nFrameWidth + dy]/1000.0<<endl;

                    //color info
                    for(int dw = 0; dw < 3; ++dw)
                    {
                        ColorImage[dx][dy][2-dw] = pColorImageRaw[(dx*nFrameWidth + dy)*3 + dw];
                    }
                }
            }

            //to cv::Mat
            cv::Mat idepth_image(nFrameHeight, nFrameWidth, CV_32F, DepthImage);
            cv::Mat ColorImage8bit(nFrameHeight, nFrameWidth, CV_8UC3, ColorImage);

            // Pass the image to the SLAM system
           SLAM.TrackRGBD(ColorImage8bit,idepth_image,EGetCurTime());
           //cv::waitKey(20);

           char key = display(ColorImage8bit );
           if(key == 'q')
           {
                if(bRestart)
                {
                    SLAM.SaveMap("ORBMap.map.bin");
                }
                break;
           }

        }

        // Stop all threads
        SLAM.Shutdown();
    }

    return;
}


int main()
{
    #ifdef Debug
    Modified(false);
    #else
    Original();
    #endif
    return 0;
}
