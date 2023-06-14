#include "./include/Stitching360.h"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <string>
#include <opencv2/videoio.hpp> 

#pragma comment(lib,"360Stitching.lib")
#define front 0
#define back 1
#define left 2
#define right 3

int main()
{
    int count=0;
    int num_c=0;
    cv::Mat total;
    total= cv::Mat::zeros(640*2,480,CV_64FC1);
    std::string path_img;

    cv::VideoCapture cap(4);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    
    if (!cap.isOpened())
    {
        printf("Can't open the camera");
        return -1;
    }

    cv::Mat mSrcFront; 
    cv::Mat mDstFront;

    while(1)
    {
        cap>>mSrcFront;

        if(count<5)
            count++; // every 5 frame, around-view image will be presented
        else
        {
            Stitching360 *stitching360= new Stitching360();
            stitching360->Init(mSrcFront.cols, mSrcFront.rows);

            // // with undistort process
            // cv::cuda::GpuMat cmDstImageFront;
            // cmDstImageFront.upload(mSrcFront);
            // cv::cuda::GpuMat cmDistortionFront = stitching360->Undistort(cmDstImageFront);
            // cmDistortionFront.download(mDstFront);
            
            // without undistort process
            mDstFront = mSrcFront; 

            // front (ahead)
            cv::Point2f mSrcPointsFront[] =
            {
                cv::Point2f(251, 341), // 4 outer corners of calibration board in image (heuristically selected image)
                cv::Point2f(520, 348),
                cv::Point2f(463, 270),
                cv::Point2f(278, 266)
            };

            cv::Point2f mDstPointsFront[] =
            {
                cv::Point2f(350, 350), // mapping points (heuristically choosen)
                cv::Point2f(150, 350),
                cv::Point2f(150, 150),
                cv::Point2f(350, 150)

            };
            cv::Mat mPerspectiveFront = stitching360->PerspectiveTransform(mDstFront, mSrcPointsFront, mDstPointsFront, cv::Size(640, 480), front);
            cv::hconcat(mSrcFront, mPerspectiveFront, total);
            count=0;
            num_c++;
            
            // path_img=".././inputImg1/"+std::to_string(num_c)+".jpg";
            // cv::imwrite(path_img,total);
            
        }

        cv::imshow("front", total);
        if (cv::waitKey(1) == 27)
            break;  // to turn off the window and break the while-loop

    }
    
}


