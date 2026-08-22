#include "myvideo.hpp"

#include <opencv2/opencv.hpp>

namespace myvideo {
MyVideo::MyVideo(const std::string &name):Node(name){
    cv::VideoCapture cap(0);
    while(1)
    {
        cv::Mat frame;
        cap>>frame;
        cv::imshow("img",frame);
        if(cv::waitKey(10)==27)
            break;
    }
}
}

int main(int argc, char **argv)
{

}