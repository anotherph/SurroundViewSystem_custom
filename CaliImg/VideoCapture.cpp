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

	int count = 0;
	int num_count = 100;


	string path_img; 
	string path = "../Img";
	bool Is_empty_; 

	while (true) 
	{
		if (Is_empty_=fs::is_empty(path)) 
		{
			path_img="../Img/1.jpg";
			break;
		}
		else
		{
			int i = 0;
            int num_file = 0;   int* num_file_ptr=&num_file;
			// int num_file_bf=0;  int* num_file_bf_ptr=&num_file_bf;
			int num_file_af=0;  int* num_file_af_ptr=&num_file_af;

			for (const auto &file : directory_iterator(path))
			{
				if (i==0)
				{
					const std::string ext = file.path().extension();
					string filename = file.path().stem();
					*num_file_ptr=stoi(filename);
					i++;
				}
				else
				{
					const std::string ext = file.path().extension();
					string filename = file.path().stem();
					// *num_file_af_ptr= (int)filename.back()-'0'; // ASCII to number
					*num_file_af_ptr = stoi(filename);
					*num_file_ptr = std::max(num_file,num_file_af);
					i++;
				}
			}
			path_img="../Img/"+to_string(num_file+1)+".jpg";
			break;
		}
	}

	while (1)
	{
		count++;
		cap >> img;
		imshow("camera img", img);

		if (count==num_count)
		{
			imwrite(path_img, img); // save the calibration image
			cout<<" image capture complete! "<<endl;
			cout<<" press ESC to turn off the image window "<<endl;
		}
		
		if (waitKey(1) == 27)
			break;  // to turn off the window and break the while moon
			
	}

	return 0;
}
