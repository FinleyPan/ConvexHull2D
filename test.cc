#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "ConvexHull.h"

#define WIDTH 960
#define HEIGHT 720
#define VALID_HEIGHT 680
#define GRAY cv::Scalar(128,128,128)
#define RED cv::Scalar(0,0,255)
#define BLUE cv::Scalar(255,0,0)
#define GREEN cv::Scalar(0,255,0)
#define YELLOW cv::Scalar(0,255,255)

struct Frame {
    cv::Mat img;
    std::vector<cv::Point2i> clicked_points;
    bool initialized = false;
    bool should_exit = true;
};

void PrintInfo(const char* msg, cv::Mat& img) {
    img(cv::Rect(0,VALID_HEIGHT,WIDTH,HEIGHT - VALID_HEIGHT)).setTo(GRAY);
    cv::putText(img, msg, cv::Point(0,VALID_HEIGHT+30),cv::FONT_HERSHEY_SIMPLEX,1.0f,GREEN);
}

void DrawPolygon(const std::vector<cv::Point>& contour2D, cv::Mat& img,const cv::Scalar& color) {
    std::vector<int> indices(contour2D.size()+1,0);
    for(int i=0; i<indices.size() - 1;i++)
        indices[i] = i;
    for(int i=0; i<indices.size() - 1 ; i++)
        cv::line(img,contour2D[indices[i]],contour2D[indices[i+1]],color);
}

void OnMouse(int events, int x,int y,int flags, void* frame) {
    cv::Mat& img = static_cast<Frame*>(frame)->img;
    auto& clicked_points = static_cast<Frame*>(frame)->clicked_points;
    bool initialized = static_cast<Frame*>(frame)->initialized;
    bool should_exit = static_cast<Frame*>(frame)->should_exit;
    if(!initialized && events == cv::MouseEventTypes::EVENT_LBUTTONDOWN) {
        if(y < VALID_HEIGHT ) {
            char coord_str[16];
            clicked_points.emplace_back(x,y);
            sprintf(coord_str,"(%d,%d)",x,y);
            cv::putText(img,coord_str,cv::Point(x,y),cv::FONT_HERSHEY_SIMPLEX,0.5,RED);
            cv::circle(img,cv::Point(x,y),2,RED);
        }
    }
    if(!should_exit && events == cv::MouseEventTypes::EVENT_LBUTTONDOWN) {
        if(y < VALID_HEIGHT) {
            char msg[] = "press carriage return once finished...";
            PrintInfo(msg,img);
            char coord_str[16];
            clicked_points.emplace_back(x,y);
            sprintf(coord_str,"(%d,%d)",x,y);
            cv::putText(img,coord_str,cv::Point(x,y),cv::FONT_HERSHEY_SIMPLEX,0.5,YELLOW);
            cv::circle(img,cv::Point(x,y),2,YELLOW);
        }
    }
}

int main(int argc, char const* const* argv) {
    typedef ConvexHull2D<int> ConvexHull;
    Frame frame;
    std::string msg;
    std::random_device random_dev;
    std::mt19937 generator(random_dev());
    std::uniform_int_distribution<int> distr(125,255);
    frame.img= cv::Mat(HEIGHT,WIDTH,CV_8UC3,GRAY);
    cv::namedWindow("disp");
    cv::setMouseCallback("disp",OnMouse,&frame);

    //make input
    msg = "click anywhere then press carriage return...";
    PrintInfo(msg.c_str(),frame.img);
    while(!frame.initialized) {
        cv::imshow("disp",frame.img);
        //break if carriage return pressed
        if(cv::waitKey(10) == 13)
           frame.initialized = true;
    }    

    //make convex hull
    ConvexHull convex_hull(frame.clicked_points);    
    DrawPolygon(convex_hull.vertices(),frame.img,BLUE);

    //expand convex hull
    frame.clicked_points.clear();
    frame.should_exit = false;
    msg = "click anywhere to expand convex hull, press Esc to exit...";
    PrintInfo(msg.c_str(),frame.img);
    while(!frame.should_exit) {
        cv::imshow(cv::String("disp"),frame.img);
        switch(cv::waitKey(1)) {
            case 13: {
                int R = distr(generator);
                int G = distr(generator);
                int B = distr(generator);
                convex_hull.ExpandAndUpdate(frame.clicked_points);
                DrawPolygon(convex_hull.vertices(),frame.img,cv::Scalar(B,G,R));
                frame.clicked_points.clear();
                msg = "click anywhere to expand convex hull, press Esc to exit...";
                PrintInfo(msg.c_str(),frame.img);
            }
                break;
            case 27:
                frame.should_exit = true; break;
            default: break;
        }

    }

    return 0;
}
