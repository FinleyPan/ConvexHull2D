#include <iostream>
#include <vector>
#include <random>

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
#define MAGENTA cv::Scalar(255,0,255)

struct Frame {
    cv::Mat img;
    std::vector<cv::Point2i> clicked_points;
    enum State {
        kZero,
        kInitialized,
        kLocateVertex,
        kExpand,
        kExit
    };

   State state = kZero;
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

void DrawPixel(int x, int y, cv::Mat& img, const cv::Scalar& color) {
    char coord_str[16];
    sprintf(coord_str,"(%d,%d)",x,y);
    cv::putText(img,coord_str,cv::Point(x,y),cv::FONT_HERSHEY_SIMPLEX,0.5,color);
    cv::circle(img,cv::Point(x,y),2,color);
}

void OnMouse(int events, int x,int y,int flags, void* frame) {
    cv::Mat& img = static_cast<Frame*>(frame)->img;
    auto& clicked_points = static_cast<Frame*>(frame)->clicked_points;    
    if(events == cv::MouseEventTypes::EVENT_LBUTTONDOWN) {
        Frame::State state = static_cast<Frame*>(frame)->state;
        switch (state) {
            case Frame::kZero:
                if(y < VALID_HEIGHT ){
                    clicked_points.emplace_back(x,y);
                    DrawPixel(x,y,img,RED);
                } break;
            case Frame::kLocateVertex:
                if(y < VALID_HEIGHT ){
                    clicked_points.emplace_back(x,y);
                    DrawPixel(x,y,img,MAGENTA);
                } break;
            case Frame::kExpand:
                if(y < VALID_HEIGHT ){
                    clicked_points.emplace_back(x,y);
                    DrawPixel(x,y,img,YELLOW);
                } break;
            case Frame::kInitialized:
            case Frame::kExit: break;
        }
    }
}

int main(int argc, char const* const* argv) {
    typedef ConvexHull2D<int> ConvexHull;
    Frame frame;
    ConvexHull convex_hull;
    std::string msg;
    frame.img= cv::Mat(HEIGHT,WIDTH,CV_8UC3,GRAY);
    cv::namedWindow("disp");
    cv::setMouseCallback("disp",OnMouse,&frame);

    //make convex hull
    msg = "click anywhere then press CR to finish...";
    PrintInfo(msg.c_str(),frame.img);
    while(frame.state == Frame::kZero) {
        cv::imshow("disp",frame.img);
        //break if carriage return pressed
        if(cv::waitKey(10) == 13) {
            if(convex_hull.ExpandAndUpdate(frame.clicked_points))
                frame.state = Frame::kInitialized;
        }
    }    
    DrawPolygon(convex_hull.contour(),frame.img,BLUE);

    const cv::Mat kBackup = frame.img.clone();
    frame.clicked_points.clear();
    while(frame.state != Frame::kExit) {
        cv::imshow(cv::String("disp"),frame.img);

        switch(cv::waitKey(1)) {
            case 8:
                frame.clicked_points.clear();
                frame.img = kBackup.clone();
                frame.state =Frame::kInitialized; break;
            case 13:
                if(frame.state == Frame::kExpand) {
                    convex_hull.ExpandAndUpdate(frame.clicked_points);
                    frame.clicked_points.clear();
                    DrawPolygon(convex_hull.contour(),frame.img,YELLOW);
                }else if(frame.state == Frame::kLocateVertex) {
                    const auto& click = frame.clicked_points.back();
                    float distance = 0.0f;
                    char coord_str[16];
                    sprintf(coord_str,"(%d,%d)",click.x,click.y);
                    msg = std::string("pixel ") + coord_str + " is";
                    ConvexHull::VertexPosition pos = convex_hull.LocateVertex(click,distance);
                    if(pos == ConvexHull::kInside) {
                       msg += " inside convex hull." ;
                    }else if(pos == ConvexHull::kAtEdge) {
                       msg += " at edge of convex hull.";
                    }else {
                        msg += " outside convex hull.";
                    }
                    PrintInfo(msg.c_str(),frame.img); break;
                } break;
            case 27:
                frame.state = Frame::kExit; break;
            case 49:
                if(frame.state == Frame::kInitialized) {
                    msg = "locate mode, press CR to finish and BS to return...";
                    PrintInfo(msg.c_str(),frame.img);
                    frame.state = Frame::kLocateVertex;
                }break;
            case 50:
                if(frame.state == Frame::kInitialized) {
                    msg = "expand mode, press CR to finish and BS to return...";
                    PrintInfo(msg.c_str(),frame.img);
                    frame.state = Frame::kExpand;
                }break;
            default:
                if(frame.state == Frame::kInitialized) {
                    msg = "press 1 to locate pixels, 2 to expand and Esc to exit...";
                    PrintInfo(msg.c_str(),frame.img);break;
                }
        }        

    }

    return 0;
}
