#define _DLL_EXPORTS
#include ".././include/Stitching360.h"
#include <opencv2/imgproc/types_c.h>
#include "opencv2/highgui.hpp"
#include "opencv2/core/cuda.hpp"
#include "opencv2/imgproc.hpp"
#define front 0
#define back 1
#define left 2
#define right 3

Stitching360::Stitching360():m_sImageRoot(".././CaliImg/Img/"), m_sLastName(".jpg"), m_sCaliResult(".././src/result.txt"), m_szBoard(cv::Size(4, 4)), m_nImageCount(10), m_nSuccessImageNum(0)
{ 
    // edit the image directory and calibration board var
}


Stitching360::~Stitching360() 
{
    cv::destroyAllWindows();
}

int Stitching360::Init(int nSrcHeight, int nSrcWidth)
{
	bool isExist = std::experimental::filesystem::exists(m_sCaliResult);
    // bool isExist = std::filesystem::exists(m_sCaliResult);
	m_szImage = cv::Size(nSrcHeight, nSrcWidth);
	if (!isExist) 
	{
		int count = findCorners();
		cameraCalibrate(count);
		savePara();
	}
	else
	{
		std::ifstream fin(m_sCaliResult);
		float d;
		int i = 0;
		// Here we also need to deal with the way of reading
		while (fin >> d) {
			if (i <= 8)
				m_mIntrinsicMatrix(i / 3, i % 3) = d;
			else if (i <= 12)
				m_vDistortionCoeffs(i - 9) = d;
			else
				m_mNewIntrinsicMat((i - 13) / 3, (i - 13) % 3) = d;
			i++;
		}
	}
	cv::Mat map1, map2;
	cv::fisheye::initUndistortRectifyMap(m_mIntrinsicMatrix, m_vDistortionCoeffs, cv::Matx33d::eye(), m_mNewIntrinsicMat, m_szImage + cv::Size(200,200), CV_32FC1, map1, map2);
    m_cmMap1.upload(map1);
	m_cmMap2.upload(map2);
    return 1;
}

int Stitching360::findCorners() 
{
	int count = 0;
	for (int i = 0; i != m_nImageCount; i++)
	{
        std::cout << "Frame #" << i + 1 << "..." << std::endl;
        std::string imageFileName;
		std::stringstream StrStm;
		StrStm << i + 1;
		StrStm >> imageFileName;
		imageFileName += m_sLastName;
		cv::Mat image = cv::imread(m_sImageRoot +  "image" + imageFileName);
		/* Extract corner points */
		cv::Mat imageGray;
		cvtColor(image, imageGray, CV_RGB2GRAY);
		/*Input image, number of corners, detected corners, adjustments made to the image before finding corners*/
		bool patternfound = cv::findChessboardCorners(image, m_szBoard, n_vCorners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE +
			cv::CALIB_CB_FAST_CHECK);

		if (!patternfound)
		{
            std::cout << "Corner not found, need to delete image file" << imageFileName << "Rearrange the filenames and recalibrate" << std::endl;
			getchar();
			exit(1);
		}
		else
		{
			/* Sub-pixel precision, refine the detected integer coordinate corner points, the refined points are stored in corners, the least square iteration is 100 times, the error is 0.001*/
			cornerSubPix(imageGray, n_vCorners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 0.001));
            std::cout << "Frame corner#" << i + 1 << "...end" << std::endl;

            // /* to check n_vCorners of each images */
            // cv::Mat temp;
            // imageGray.copyTo(temp);
            // for (int j=0; j<49; ++j)
            // { cv::circle(temp,n_vCorners[j],5,cv::Scalar(255,0,255),3,8,0);  }
            // cv::imshow("temp",temp);
            // cv::waitKey();

			count = count + n_vCorners.size();
			m_nSuccessImageNum = m_nSuccessImageNum + 1;
			m_vCornersSeq.push_back(n_vCorners);
		}
		m_vImageSeq.push_back(image);
	}
	return count;
    std::cout << "Corner point extraction complete!\n";
    return 1;
}

int Stitching360::cameraCalibrate(int count) {
    std::cout << "Start calibration……………" << std::endl;
	cv::Size square_size = cv::Size(20, 20); /**** Each grid is 20m*20mm ****/
    std::vector<std::vector<cv::Point3f>>  object_Points; /**** Save the three-dimensional coordinates of the corner points on the calibration board ****/

	cv::Mat image_points = cv::Mat(1, count, CV_32FC2, cv::Scalar::all(0));  /***** Save all extracted corners *****/
    std::vector<int>  point_counts;
	/* Initialize the three-dimensional coordinates of the corner points on the calibration board */
	for (int t = 0; t<m_nSuccessImageNum; t++)
	{
        std::vector<cv::Point3f> tempPointSet;
		for (int i = 0; i<m_szBoard.height; i++)
		{
			for (int j = 0; j<m_szBoard.width; j++)
			{
				/* Assume that the calibration board is placed on the plane of z=0 in the world coordinate system */
				cv::Point3f tempPoint;
				tempPoint.x = i*square_size.width;
				tempPoint.y = j*square_size.height;
				tempPoint.z = 0;
				tempPointSet.push_back(tempPoint);
			}
		}
		object_Points.push_back(tempPointSet);
	}
	for (int i = 0; i< m_nSuccessImageNum; i++)
	{
		point_counts.push_back(m_szBoard.width*m_szBoard.height);
	}
	/* start calibration */
	cv::Size image_size = m_vImageSeq[0].size();
	int flags = 0;
	flags |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
	flags |= cv::fisheye::CALIB_CHECK_COND;
	flags |= cv::fisheye::CALIB_FIX_SKEW;
	/* The coordinates of the corner points in the calibration board, the coordinates of the corner points in the image, 
    the size of the image, the internal reference matrix, the four distortion parameters, the output rotation vector, 
    the output translation vector, the number of iterations, the error*/
	cv::fisheye::calibrate(object_Points, m_vCornersSeq, image_size, m_mIntrinsicMatrix, m_vDistortionCoeffs, m_vRotationVectors, m_vTranslationVectors, flags, cv::TermCriteria(3, 20, 1e-6));
	cv::fisheye::estimateNewCameraMatrixForUndistortRectify(m_mIntrinsicMatrix, m_vDistortionCoeffs, image_size, cv::noArray(), m_mNewIntrinsicMat, 0.8, image_size, 1.0);
    std::cout << "Calibration completed!\n";
    return 1;
}

int Stitching360::savePara() {
    std::cout << "Start saving calibration results............." << std::endl;
    std::ofstream fout(m_sCaliResult);
	/*Camera Intrinsic Parameter Matrix*/
    fout << m_mIntrinsicMatrix(0,0) << ' ' << m_mIntrinsicMatrix(0, 1) << ' ' << m_mIntrinsicMatrix(0, 2) << ' ' << m_mIntrinsicMatrix(1, 0) << ' ' << m_mIntrinsicMatrix(1, 1)
		<< ' ' << m_mIntrinsicMatrix(1, 2) << ' ' << m_mIntrinsicMatrix(2, 0) << ' ' << m_mIntrinsicMatrix(2, 1) << ' ' << m_mIntrinsicMatrix(2, 2) << std::endl;
	/*Distortion coefficient*/
	fout << m_vDistortionCoeffs(0) << ' '<< m_vDistortionCoeffs(1) <<' '<< m_vDistortionCoeffs(2) << ' ' << m_vDistortionCoeffs(3) << std::endl;
	/*Correction internal reference matrix*/
	fout << m_mNewIntrinsicMat(0, 0) << ' ' << m_mNewIntrinsicMat(0, 1) << ' ' << m_mNewIntrinsicMat(0, 2) << ' ' << m_mNewIntrinsicMat(1, 0) << ' ' << m_mNewIntrinsicMat(1, 1)
		<< ' ' << m_mNewIntrinsicMat(1, 2) << ' ' << m_mNewIntrinsicMat(2, 0) << ' ' << m_mNewIntrinsicMat(2, 1) << ' ' << m_mNewIntrinsicMat(2, 2) << std::endl;
    std::cout << "saving done" << std::endl;
	fout << std::endl;
    return 1;
}

cv::cuda::GpuMat Stitching360::Undistort(cv::cuda::GpuMat &mSrcImg)
{
	cv::cuda::GpuMat mDstImg; 
    cv::Mat mDstImg_, mSrcImg_, m_cmMap1_, m_cmMap2_;  

    // to compute cv::remap, variables on GPU must be downloaded into CPU
    mSrcImg.download(mSrcImg_);
    m_cmMap1.download(m_cmMap1_);
    m_cmMap2.download(m_cmMap2_);
    
    cv::remap(mSrcImg_, mDstImg_, m_cmMap1_, m_cmMap2_, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

    mDstImg.upload(mDstImg_);
    cv::imwrite("./Src.jpg",mSrcImg_);
    cv::imwrite("./Dst.jpg",mDstImg_);

    return mDstImg;

 }

cv::Mat Stitching360::PerspectiveTransform(cv::InputArray aInput, cv::Point2f *pSrcPoints, cv::Point2f *pDstPoints, cv::Size sOutputSize, int nOrientation) 
{
    cv::Mat mPerspective = cv::getPerspectiveTransform(pSrcPoints, pDstPoints);
    cv::Mat mPerspectiveImg;
    cv::warpPerspective(aInput, mPerspectiveImg, mPerspective, sOutputSize, cv::INTER_LINEAR);
    if (nOrientation == front)
    {

    }
    else if (nOrientation == back)
    {
        cv::flip(mPerspectiveImg, mPerspectiveImg, -1);
    }
    else if (nOrientation == left)
    {
        cv::transpose(mPerspectiveImg, mPerspectiveImg);
        cv::flip(mPerspectiveImg, mPerspectiveImg, 0);
    }
    else if (nOrientation == right)
    {
        cv::transpose(mPerspectiveImg, mPerspectiveImg);
        cv::flip(mPerspectiveImg, mPerspectiveImg, 1);
    }

    return mPerspectiveImg;
}

cv::Mat Stitching360::ImageStitching(int nWidth, int nHeight, cv::Mat mInputLeft, cv::Mat mInputRight, cv::Mat mInputFront, cv::Mat mInputBack,
    std::vector<cv::Point> vPtsLeft, std::vector<cv::Point> vPtsRight, std::vector<cv::Point> vPtsFront, std::vector<cv::Point> vPtsBack)
{
    cv::Mat mCombine = cv::Mat::zeros(1600, 1600, mInputRight.type());
    cv::Mat mRoiInputRight = cv::Mat::zeros(mInputRight.size(), CV_8U);
    cv::Mat mRoiInputLeft = cv::Mat::zeros(mInputLeft.size(), CV_8U);
    cv::Mat mRoiInputFront = cv::Mat::zeros(mInputFront.size(), CV_8U);
    cv::Mat mRoiInputBack = cv::Mat::zeros(mInputBack.size(), CV_8U);

    std::vector<cv::Point> vStitchFront;
    std::vector<cv::Point> vStitchBack;

    std::vector<std::vector<cv::Point>> vContourInputRight;
    std::vector<std::vector<cv::Point>> vContourInputLeft;
    std::vector<std::vector<cv::Point>> vContourInputFront;
    std::vector<std::vector<cv::Point>> vContourInputBack;

    cv::Mat mImgRoiInputFront;
    cv::Mat mImgRoiInputBack;
    cv::Mat mImgRoiInputLeft;
    cv::Mat mImgRoiInputRight;

    /***********************************to cut**************************************************/
    // front cutting
    vStitchFront.push_back(cv::Point(vPtsFront.at(1).x - nHeight + vPtsFront.at(1).y, nHeight));
    vStitchFront.push_back(cv::Point(vPtsFront.at(0).x + nHeight - vPtsFront.at(0).y, nHeight));
    if (vPtsFront.at(0).x > vPtsFront.at(0).y)
    {
        vStitchFront.push_back(cv::Point(vPtsFront.at(0).x - vPtsFront.at(0).y, 0));
    }
    else
    {
        vStitchFront.push_back(cv::Point(0, vPtsFront.at(0).y - vPtsFront.at(0).x));
        vStitchFront.push_back(cv::Point(0, 0));
    }
    if (nWidth - vPtsFront.at(1).x > vPtsFront.at(1).y)
    {
        vStitchFront.push_back(cv::Point(vPtsFront.at(1).x + vPtsFront.at(0).y, 0));
    }
    else
    {
        vStitchFront.push_back(cv::Point(nWidth, 0));
        vStitchFront.push_back(cv::Point(nWidth, vPtsFront.at(1).y - nWidth + vPtsFront.at(1).x));
    }

    // back cutting
    vStitchBack.push_back(cv::Point(vPtsBack.at(1).x - vPtsBack.at(1).y, 0));
    vStitchBack.push_back(cv::Point(vPtsBack.at(0).x + vPtsBack.at(0).y, 0));
    if (nHeight - vPtsBack.at(0).y < vPtsBack.at(0).x)
    {
        vStitchBack.push_back(cv::Point(vPtsBack.at(0).x - nHeight + vPtsBack.at(0).y, nHeight));
    }
    else
    {
        vStitchBack.push_back(cv::Point(0, vPtsBack.at(0).x + vPtsBack.at(0).y));
        vStitchBack.push_back(cv::Point(0, nHeight));
    }
    if (nWidth - vPtsBack.at(1).x > vPtsBack.at(1).y)
    {
        vStitchBack.push_back(cv::Point(vPtsBack.at(1).x + nHeight - vPtsBack.at(1).y, nHeight));
    }
    else
    {
        vStitchBack.push_back(cv::Point(nWidth, nHeight));
        vStitchBack.push_back(cv::Point(nWidth, vPtsBack.at(1).y - nWidth + vPtsBack.at(1).x));
    }


    /*****************************computing edge************************************/
    int nDiffFL_x = vPtsLeft.at(0).x - vPtsFront.at(0).x;
    int nDiffBL_x = vPtsLeft.at(1).x - vPtsBack.at(0).x;
    if (nDiffFL_x < nDiffBL_x)
    {
        if (nDiffFL_x <= 0)
        {
            mImgRoiInputFront = mCombine(cv::Rect(0, 0, nWidth, nHeight));
            mImgRoiInputLeft = mCombine(cv::Rect(-nDiffFL_x, vPtsFront.at(0).y - vPtsLeft.at(0).y, nHeight, nWidth));
            mImgRoiInputBack = mCombine(cv::Rect(-nDiffFL_x + nDiffBL_x, vPtsFront.at(0).y + vPtsLeft.at(1).y - vPtsLeft.at(0).y - vPtsBack.at(0).y, nWidth, nHeight));
            mImgRoiInputRight = mCombine(cv::Rect(vPtsLeft.at(0).x + vPtsFront.at(1).x - vPtsFront.at(0).x - vPtsRight.at(0).x - nDiffFL_x, vPtsFront.at(1).y - vPtsRight.at(0).y, nHeight, nWidth));
        }
        else
        {
            mImgRoiInputFront = mCombine(cv::Rect(nDiffFL_x, 0, nWidth, nHeight));
            mImgRoiInputLeft = mCombine(cv::Rect(0, vPtsLeft.at(0).y - vPtsFront.at(0).y, nHeight, nWidth));
            mImgRoiInputBack = mCombine(cv::Rect(nDiffBL_x, vPtsFront.at(0).y + vPtsLeft.at(1).y - vPtsLeft.at(0).y - vPtsBack.at(0).y, nWidth, nHeight));
            mImgRoiInputRight = mCombine(cv::Rect(vPtsLeft.at(0).x + vPtsFront.at(1).x - vPtsFront.at(0).x - vPtsRight.at(0).x, vPtsFront.at(1).y - vPtsRight.at(0).y, nHeight, nWidth));
        }
    }
    else
    {
        if (nDiffBL_x <= 0)
        {
            mImgRoiInputFront = mCombine(cv::Rect(-nDiffBL_x + nDiffFL_x, 0, nWidth, nHeight));
            mImgRoiInputLeft = mCombine(cv::Rect(-nDiffBL_x, vPtsFront.at(0).y - vPtsLeft.at(0).y, nHeight, nWidth));
            mImgRoiInputBack = mCombine(cv::Rect(0, vPtsFront.at(0).y + vPtsLeft.at(1).y - vPtsLeft.at(0).y - vPtsBack.at(0).y, nWidth, nHeight));
            mImgRoiInputRight = mCombine(cv::Rect(vPtsLeft.at(0).x + vPtsFront.at(1).x - vPtsFront.at(0).x - vPtsRight.at(0).x - nDiffBL_x, vPtsFront.at(1).y - vPtsRight.at(0).y, nHeight, nWidth));
        }
        else
        {
            mImgRoiInputFront = mCombine(cv::Rect(nDiffFL_x, 0, nWidth, nHeight));
            mImgRoiInputLeft = mCombine(cv::Rect(0, vPtsLeft.at(0).y - vPtsFront.at(0).y, nHeight, nWidth));
            mImgRoiInputBack = mCombine(cv::Rect(nDiffBL_x, vPtsFront.at(0).y + vPtsLeft.at(1).y - vPtsLeft.at(0).y - vPtsBack.at(0).y, nWidth, nHeight));
            mImgRoiInputRight = mCombine(cv::Rect(vPtsLeft.at(0).x + vPtsFront.at(1).x - vPtsFront.at(0).x - vPtsRight.at(0).x, vPtsFront.at(1).y - vPtsRight.at(0).y, nHeight, nWidth));
        }
    }

    vContourInputBack.push_back(vStitchBack);
    drawContours(mRoiInputBack, vContourInputBack, 0, cv::Scalar::all(255), -1);

    vContourInputFront.push_back(vStitchFront);
    drawContours(mRoiInputFront, vContourInputFront, 0, cv::Scalar::all(255), -1);
    
    // Put left and right first, because there is no cutting. Front and back are cut
    mInputRight.copyTo(mImgRoiInputRight);
    mInputLeft.copyTo(mImgRoiInputLeft);
    mInputFront.copyTo(mImgRoiInputFront, mRoiInputFront);
    mInputBack.copyTo(mImgRoiInputBack, mRoiInputBack);
    //mCombine = mCombine(cv::Rect(0, 0, 900, 910));
    return mCombine;
}


// extern "C" DLL_API SurroundView *GetStitching()
// {
//     return new Stitching360;
// }