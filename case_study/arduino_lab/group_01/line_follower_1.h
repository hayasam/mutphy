#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

enum states {Straight, SS_Left, SS_Right, S_Left, S_Right, Turn_Left, Turn_Right};
int state;

float engine_right;
float engine_left;

void translate_contour(vector<Point> &contour, int x, int y);

float getMidPoint(Mat image);

void imageCallback(const sensor_msgs::ImageConstPtr& msg);

//This function updates the engine right/left values which are being sent to the robot,
//Contour as input argument previously calculated from the image
void centerToEngine(float center);

void automata_engines();
