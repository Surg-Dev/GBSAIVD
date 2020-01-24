/// Individual Include Files, refine later
//#include <opencv2/core.hpp>
//#include <opencv2/imgproc.hpp>
//#include <opencv2/videoio.hpp>
//#include <opencv2/highgui.hpp>

//Include all of opencv and cuda libraries (seperate)
#include <opencv2/opencv.hpp>
#include "opencv2/cudaimgproc.hpp"

// Standard cpp libraries, equivilents are like numpy
#include <iostream>
#include <cmath>

//Using namespaces, std can usually be ignorned, but cv and cv::cuda have overloaded functions
using namespace std;
using namespace cv;
using namespace cv::cuda;

// Default pipeline with variable passthrough
std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! appsink";
}

//Pipeline without variable passthrough
//std::string altpipeline="nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)NV12, framerate=(fraction)15/1 ! nvvidconv flip-method=0 ! video/x-raw, width=(int)1280, height=(int)720, format=(string)BGRx ! videoconvert ! appsink";

int main(){
    //Define variables for pipeline. Min Capture Dims are 1280x720@120fps, but Display (matrix pulls) are at 640x360@10fps
    int capture_width = 1280 ;
    int capture_height = 720 ;
    int display_width = 640 ;
    int display_height = 360 ;
    int framerate = 10 ;
    int flip_method = 0 ;
    
    //Pull gstreamer pipeline to make a string showing what parameters it is using in console
    std::string pipeline = gstreamer_pipeline(capture_width,
	capture_height,
	display_width,
	display_height,
	framerate,
	flip_method);
    std::cout << "Using pipeline: \n\t" << pipeline << "\n";

    //Capture video using the pipeline + gstreamer
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

    //If cap isnt opened, it doesnt latch, so it exits, as there is a seperate problem.
    if(!cap.isOpened()) {
	std::cout<<"Failed to open camera."<<std::endl;
	return (-1);
    }

    //Define display windows, temporary, not needed for headless
    cv::namedWindow("CSI Camera", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Line Camera", cv::WINDOW_AUTOSIZE);
    
    //Define source Matrix
    cv::Mat src;

    //Video loop
    while (true){

        //Reads and saves the matrix to src. Checks if empties and exits if it fails
    	if (!cap.read(src)) {
		    std::cout<<"Capture read error"<<std::endl;
		    break;
        }

        //cap.read(src);
        
        //Define a gray matrix, load up grayscale version of src to gray
        cv::Mat gray;
        cv::cvtColor(src,gray,COLOR_BGR2GRAY);
        //Define a mask matrix, load canny edge detection of gray to mask
        cv::Mat mask;
        Canny(gray, mask, 100, 200, 3);
        //Define the GPU vision matrix, make it from mask to a BGR (B+W w/ Colored lines)
        cv::Mat dst_gpu;
        cv::cvtColor(mask, dst_gpu, COLOR_GRAY2BGR);
        //Define the GPU Matrix, based off of mask
        GpuMat d_src(mask);
        
        //Define the line detection GPU Matrix
        GpuMat d_lines;
        {
            //Create the pointer of the segment detector on the GPU
            Ptr<cuda::HoughSegmentDetector> hough = cuda::createHoughSegmentDetector(1.0f, (float) (CV_PI / 180.0f), 50, 5);
            //Tell the pointer to detect lines from d_src GpuMat to d_lines GpuMat
            hough->detect(d_src, d_lines);

            /* We can't tell the GPU to "do this" so we have to create pointers to objects, 
            then tell those pointers to do something, and look for the return
            */
        }
        //Create a vector of Vec4i for the gpu lines
        vector<Vec4i> lines_gpu;
        if (!d_lines.empty()) //If populated with detected lines
        {
            //Resize the vector to the width (?) of the d_lines matrix
            lines_gpu.resize(d_lines.cols);
            //Create a generic CPU matrix  populated with the lines_gpu
            Mat h_lines(1, d_lines.cols, CV_32SC4, &lines_gpu[0]);
            //Download the d_lines GPU matrix to the h_lines CPU matrix
            d_lines.download(h_lines);
            /*  We can't access the memory of the GPU directly (It won't let us),
                So we must download the matrix from the GPU.
            */
        }

        for (size_t i = 0; i < lines_gpu.size(); ++i)
        {   
            //Create a vector of the first line vector in the lines_gpu vector array
            Vec4i l = lines_gpu[i];
            //Draw the vector using a BGR Scalar onto the dst_gpu matrix
            line(dst_gpu, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, LINE_AA);
        }

            cv::imshow("CSI Camera",src); //Show the original video
            cv::imshow("Line Camera",dst_gpu); // Show the line detection video
	        int keycode = cv::waitKey(30) & 0xff ; //Break key logic.
            if (keycode == 27) break ;

        }
    cap.release(); //IMPORTANT! This must run before exiting any cpp program, or the camera will get stuck
    cv::destroyAllWindows() ; //Closes windows
    return 0; //Returns with no intended error.

}