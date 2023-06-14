#pragma once

// #if defined(_DLL_EXPORTS) // inside DLL
// #   define DLL_API   __declspec(dllexport)
// #else // outside DLL
// #   define DLL_API   __declspec(dllimport)
// #endif  // XYZLIBRARY_EXPORT

// #include <opencv2\opencv.hpp>
#include <opencv2/opencv.hpp>
//#include </usr/local/include/opencv4/opencv2/opencv.hpp>
#include <experimental/filesystem>
// #include <filesystem>
#include <fstream>

class SurroundView
{
public:
    /************************Camera Calibration and Correction****************************/
    virtual int Init(int nSrcHeight, int nSrcWidth) = 0;
    virtual cv::cuda::GpuMat Undistort(cv::cuda::GpuMat &mSrcImg) = 0;

    /************************back projection transformation*******************************/
    virtual cv::Mat PerspectiveTransform(cv::InputArray aInput, cv::Point2f *pSrcPoints, cv::Point2f *pDstPoints, cv::Size sOutputSize, int nOrientation) = 0;

    /*************************image stitching**********************************/
    virtual cv::Mat ImageStitching(int nWidth, int nHeight, cv::Mat aInputLeft, cv::Mat aInputRight, cv::Mat aInputFront, cv::Mat aInputBack,
        std::vector<cv::Point> vPtsInputLeft, std::vector<cv::Point> vPtsInputRight, std::vector<cv::Point> vPtsInputFront, std::vector<cv::Point> vPtsInputBack) = 0;

};

class Stitching360 :public SurroundView
{
private:
    std::string                             m_sImageRoot;	        /* Image forder */
    std::string                             m_sLastName;            /* Image extension */
    std::string                             m_sCaliResult;          /* directory where the calibration data is stored*/
    cv::Size                                m_szImage;
    cv::Size                                m_szBoard;	            /**** The number of corner points in each row and column on the calibration board       ****/
    int                                     m_nImageCount;	        /**** Number of calibration images ****/
    int                                     m_nSuccessImageNum;     /**** Number of checkerboard diagrams with successfully extracted corner points    ****/
    cv::Matx33d                             m_mIntrinsicMatrix;     /**** Camera Intrinsic Parameter Matrix ****/
    cv::Matx33d                             m_mNewIntrinsicMat;     /** The new internal parameters of the camera are used for correction **/
    cv::Vec4d                               m_vDistortionCoeffs;    /* 4 distortion coefficients of the camera: k1,k2,k3,k4*/
    std::vector<cv::Mat>                    m_vImageSeq;		    /* save image */
    std::vector<std::vector<cv::Point2f>>   m_vCornersSeq;          /**** Save all detected corners ****/
    std::vector<cv::Point2f>                n_vCorners;             /**** Cache the corners detected on each image ****/
    std::vector<cv::Vec3d>                  m_vRotationVectors;     /* rotation vector for each image */
    std::vector<cv::Vec3d>                  m_vTranslationVectors;  /* translation vector for each image */
    cv::cuda::GpuMat                        m_cmMap1;               /* Final Corrected Mapping Table, the first output map */
    cv::cuda::GpuMat                        m_cmMap2;               /* Final Corrected Mapping Table, the second output map */

    int findCorners();
    int cameraCalibrate(int count);
    int savePara();
    void OnMouseAction(int event, int x, int y, int flags, void *para);


public:
    Stitching360();
    ~Stitching360();
    /************************Camera Calibration and Correction****************************/
    virtual int Init(int nSrcHeight, int nSrcWidth);
    virtual cv::cuda::GpuMat Undistort(cv::cuda::GpuMat &mSrcImg);

    /************************back projection transformation*******************************/
    virtual cv::Mat PerspectiveTransform(cv::InputArray aInput, cv::Point2f *pSrcPoints, cv::Point2f *pDstPoints, cv::Size sOutputSize, int nOrientation);

    /*************************image stitching**********************************/
    virtual cv::Mat ImageStitching(int nWidth, int nHeight, cv::Mat aInputLeft, cv::Mat aInputRight, cv::Mat aInputFront, cv::Mat aInputBack,
        std::vector<cv::Point> vPtsInputLeft, std::vector<cv::Point> vPtsInputRight, std::vector<cv::Point> vPtsInputFront, std::vector<cv::Point> vPtsInputBack);

};

// extern "C" DLL_API SurroundView *GetStitching();
