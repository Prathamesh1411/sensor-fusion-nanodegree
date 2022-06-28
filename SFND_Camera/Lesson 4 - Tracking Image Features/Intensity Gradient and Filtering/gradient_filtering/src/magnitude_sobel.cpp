#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

void magnitudeSobel()
{
    // load image from file
    cv::Mat img;
    img = cv::imread("../images/img1gray.png");

    // convert image to grayscale
    cv::Mat imgGray;
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

    // apply smoothing using the GaussianBlur() function from the OpenCV
    // ToDo : Add your code here
    // cv::Mat imgBlur;
    // cv::GaussianBlur(imgGray, imgBlur, cv::Size(5,5), 0, 0);

    cv::Mat imgBlur = imgGray.clone();
    int filterSize = 5;
    int stdDev = 2.0;
    cv::GaussianBlur(imgGray, imgBlur, cv::Size(filterSize, filterSize), stdDev);

    // create filter kernels using the cv::Mat datatype both for x and y
    // ToDo : Add your code here
    float sobelX[9] = {-1, 0, +1,
                        -2, 0, +2,
                        -1, 0, +1};

    float sobelY[9] = {-1, -2, -1,
                        0, 0, 0,
                        +1, +2, +1};
                        
    cv::Mat kernelX = cv::Mat(3, 3, CV_32F, sobelX);
    cv::Mat kernelY = cv::Mat(3, 3, CV_32F, sobelY);

    // apply filter using the OpenCv function filter2D()
    // ToDo : Add your code here
    cv::Mat filterX;
    cv::Mat filterY;
    cv::filter2D(imgBlur, filterX, -1, kernelX, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
    cv::filter2D(imgBlur, filterY, -1, kernelY, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);

    // compute magnitude image based on the equation presented in the lesson 
    // ToDo : Add your code here
    cv::Mat magnitude = imgGray.clone();
    for(int r = 0; r < magnitude.rows; r++){
        for(int c = 0; c < magnitude.cols; c++){
            magnitude.at<unsigned char>(r, c) = sqrt(pow(filterX.at<unsigned char>(r, c), 2)+
                                                    pow(filterY.at<unsigned char>(r, c), 2));
        }
    }

    // show result
    string windowName = "Gaussian Blurring";
    cv::namedWindow(windowName, 1); // create window
    cv::imshow(windowName, magnitude);
    cv::waitKey(0); // wait for keyboard input before continuing
}

int main()
{
    magnitudeSobel();
}