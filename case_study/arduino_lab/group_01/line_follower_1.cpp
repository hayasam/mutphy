/**
 * Group number: 1
 * Student 1:
 * Stefan Breetveld, 4374657
 * Student 2:
 * David Viteri, 4580958
 */

/*
 * Initial setup requires a few commands to be run.
 * 1. ifconfig | get the <ip> adress of this pc.
 * 2. ROS_IP=<ip>  | set the variable ROSIP to the current ip adress.
 * 3. export ROS_IP  | set the environment variable so you don't have to set it again.
 * 4. ROS_MASTER_URI=http://$ROS_IP:11311/ | set the URI where ros should listen for communication.
 * 5. export ROS_MASTER_URI  | same as before.
 *
 * Before running make sure to source the workspace 
 * . <ws_dir>/devel/setup.bash
 *
 * To be able to get an image from the camera we need to use the compressed image transport type
 *   to do so use the following commands.
 * rosparam set /compressed_listener/image_transport compressed
 * rosrun line_follower line_follower __name:=compressed_listener
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdlib.h>
#include "line_follower_1.h"

//Use the opencv namespace so we don't have to type cv:: everytime we want to use a function or type from opencv
using namespace cv; 
using namespace std; 

//this should be larger than the maximum width of the image.
const double large_number = 1000000; 
const char* window_name_1 = "src";
const char* window_name_2 = "detected";
const char* window_name_3 = "thresh";
const char* window_name_4 = "gray";

int erosion_size = 11;
int dilation_size = 11;

const float low_speed = 50;
const float mid_speed = 75;
const float top_speed = 100;

ros::Publisher pub;

int main(int argc, char **argv)
{
  // initialise ROS
  ros::init(argc, argv, "line_follower"); 
  // initialise ROS nodehandler
  ros::NodeHandle nh; 
  // tell ROS that we are going to publish messages on /cmd_vel of type geomety_msgs/Twist
  pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1); 
  // create a window that shows the source image
  namedWindow(window_name_1, CV_WINDOW_NORMAL);
  // create a window that shows the region, contours and the followed line 
  namedWindow(window_name_2, CV_WINDOW_NORMAL); 
  // create a window that shows the image that results from the threshold
  namedWindow(window_name_3, CV_WINDOW_NORMAL); 
  // create a window that shows the grayscale of the source image
  namedWindow(window_name_4, CV_WINDOW_NORMAL); 

  startWindowThread();
  image_transport::ImageTransport it(nh);
  // Tell ROS we want to subscribe to messages on /camera/image
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback); 
  // Tell ROS to keep looking for new messages.
  ros::spin(); 

  destroyWindow(window_name_1);
  destroyWindow(window_name_2);
  destroyWindow(window_name_3);
  destroyWindow(window_name_4);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    // Create a local copy of the image we received so that we are allowed to edit it. 
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8"); 

    // Rotate image 90 degrees counter clockwise.
    transpose(cv_ptr->image, cv_ptr->image);  
    flip(cv_ptr->image, cv_ptr->image,1);

    // Find something we can follow
    float line = getMidPoint(cv_ptr->image.clone());
    // Convert the found value to values we can send to the robot
    centerToEngine(line);

    //Publish msg
    geometry_msgs::Twist msg;
  	msg.linear.x = engine_right;
  	msg.linear.y = engine_left;
  	pub.publish(msg);

    // Show the source image
    imshow(window_name_1, cv_ptr->image);
    
    waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

float distToCenter(Point2f point, Mat image) 
{
  return abs(((float) image.cols) / 2 - point.x);
}

float getMidPoint(Mat image) 
{
  Mat mono, blur, thresh, erodeImg, dilateImg; 
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  // Define our region of interest
  int top_left_x = 0;
  int top_left_y = image.rows / 4 * 3;
  int height = image.rows / 4;
  int width = image.cols - 2 * top_left_x;
  Rect rect = Rect(top_left_x, top_left_y, width, height);
  Mat roi(image, rect);

  // Convert to grayscale
  cvtColor(roi, mono, CV_BGR2GRAY);
  // Show grayscale image
  imshow(window_name_4, mono);
  // Blur the gray image
  GaussianBlur(mono, blur, Size(9, 9), 2, 2);
  // Apply an inverse binary otsu threshold
  threshold(blur, thresh, 0, 255, CV_THRESH_BINARY_INV|CV_THRESH_OTSU);
  // Show the result of the threshold
  imshow(window_name_3, thresh);

  // Remove small lines or areas from the image, but keep everything else the same size
  Mat erode_element = getStructuringElement(MORPH_RECT,
                                     Size(2*erosion_size + 1, 2*erosion_size+1 ),
                                     Point(erosion_size, erosion_size));
  Mat dilate_element = getStructuringElement(MORPH_RECT,
                                     Size(2*dilation_size + 1, 2*dilation_size+1),
                                     Point(dilation_size, dilation_size));
  erode(thresh, erodeImg, erode_element);
  dilate(erodeImg, dilateImg, dilate_element);

  // Find contours in the image.
  findContours(dilateImg, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

  double dist = large_number;
  int offset = roi.cols / 2;
  
  // Get the moments
  vector<Moments> mu(contours.size());
  for( int i = 0; i < contours.size(); i++ )
     { mu[i] = moments( contours[i], false ); }

  //  Get the centers of mass
  vector<Point2f> mc( contours.size() );
  for( int i = 0; i < contours.size(); i++ )
     { mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }

  // overlay contours on source image.
  for(int i = 0; i < contours.size(); i++)
  {
    Scalar color = Scalar( 0, 255, 0);
    translate_contour(contours[i], top_left_x, top_left_y);
    drawContours( image, contours, i, color, 2, 8, hierarchy, 0, Point() );
  }

  // Find center of mass closest to the center of the image
  for(int i = 0; i < mc.size(); i++)
  {
    Point2f contour_center = mc[i];
    float temp_dist = distToCenter(contour_center, roi) ;
    if(temp_dist < dist) {
        offset = contour_center.x;
        dist = temp_dist;
    }
  }

  // Overlay roi box on source image
  rectangle(image, rect, Scalar(255, 0, 0));
  // Overlay line representing the center of mass.
  line(image, Point(top_left_x + offset, top_left_y), Point(top_left_x + offset, top_left_y + height), Scalar(0, 0, 255), 2, 8, 0);
  // Show the resulting image
  imshow(window_name_2, image);
  // Convert the found center of mass to a number betweer minus one and one. minus one being far left and one being far right.
  return -1.0f + 2.0f * (float) offset / roi.cols;
}

// Translate a contour by the given x and y values.
void translate_contour(vector<Point> &contour, int x, int y)
{
  for (size_t i=0; i<contour.size(); i++)
  {
    contour[i].x += x;
    contour[i].y += y;
  }
}

// Convert a number betweer minus one and one to engine states.
void centerToEngine(float center){
	if(center < -0.8)
		state = SS_Left;
	else if(center < -0.6)
		state = S_Left;
	else if(center < -0.2)
		state = Turn_Left;
	else if(center < 0.2)
		state = Straight;
	else if(center < 0.6)
		state = Turn_Right;
	else if(center < 0.8)
		state = S_Right;
	else if(center < 1.0)
		state = SS_Right;

	automata_engines();
}

// Convert engine states to engine speeds.
void automata_engines(){
	switch (state){
		case Straight:
			engine_left = mid_speed;
			engine_right = mid_speed;
			break;
    case Turn_Left:
      engine_left = low_speed;
      engine_right = mid_speed;
      break;
    case Turn_Right:
      engine_left = mid_speed;
      engine_right = low_speed;
      break;
		case S_Left:
			engine_left = 0.0;
			engine_right = mid_speed;
			break;
		case S_Right:
			engine_left = mid_speed;
			engine_right = 0.0;
			break;
    case SS_Left:
      engine_left = 0.0;
      engine_right = top_speed;
      break;
    case SS_Right:
      engine_left = top_speed;
      engine_right = 0.0;
      break;
	}

}






