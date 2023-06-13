#include <experimental/filesystem>
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp> 

using namespace cv;
using namespace std;
using std::experimental::filesystem::directory_iterator;
namespace fs = std::experimental::filesystem;

int main ()
{
    VideoCapture cap(4);
	cap.set(CAP_PROP_FRAME_WIDTH, 640);
	cap.set(CAP_PROP_FRAME_HEIGHT, 480);
	
    if (!cap.isOpened())
    {
        printf("Can't open the camera");
        return -1;
    }
    
    Mat img;

	// keep going until esc...
	// while (1)
	// {
	// 	cap >> img;

	// 	imshow("camera img", img);
	// 	// cap>>img;
	// 	imwrite("image.jpg", img);

	// 	if (waitKey(1) == 27)
	// 		break;
	// }

	int count = 0;
	int num_count = 100;


	string path_img; 
	string path = "../Img";
	bool Is_empty_; 

	while (true) 
	{
		if (Is_empty_=fs::is_empty(path)) 
		{
			path_img="../Img/image1.jpg";
			break;
		}
		else
		{
			int i = 0;
            int num_file = 0;   int* num_file_ptr=&num_file;
			int num_file_bf=0;  int* num_file_bf_ptr=&num_file_bf;
			int num_file_af=0;  int* num_file_af_ptr=&num_file_af;

			for (const auto &file : directory_iterator(path))
			{
				if (i==0)
				{
					const std::string ext = file.path().extension();
					string filename = file.path().stem();
					*num_file_bf_ptr= (int)filename.back()-'0'; // ASCII to number
					i++;
				}
				else
				{
					const std::string ext = file.path().extension();
					string filename = file.path().stem();
					*num_file_af_ptr= (int)filename.back()-'0'; // ASCII to number
					*num_file_ptr = std::max(num_file_bf,num_file_af);
					*num_file_bf_ptr= *num_file_af_ptr;
					i++;
				}
			}
			path_img="../Img/image"+to_string(num_file+1)+".jpg";
			break;
		}
	}

	while (count<num_count)
	{
		count++;
		cap >> img;
		//imshow("camera img", img);
		
		if (count==num_count)
			{
				imwrite(path_img, img);
			}
	}

	return 0;
}
