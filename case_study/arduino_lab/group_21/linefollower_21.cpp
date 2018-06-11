/**
  * Group number: 21
  * Student 1:
  * Daan van der Valk, 4094751
  * Student 2:
  * Stefan de Vringer, 4374851
  */

// Includes required for ROS and the image processing
#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/CompressedImage.h"
#include "math.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;

// Name of debugging window
static const std::string OPENCV_WINDOW = "Linefollower debugger by team 21";
// Declare publisher
ros::Publisher pub;

// Function to process the incoming images and send Twist message if a path can be found 
void imageCallback(const cv_bridge::CvImage::ConstPtr& msg)
{
  // Image processing
  // msg->image points to a cv::Mat object
  Mat img = msg->image.clone();
  
  // Convert to greyscale
  Mat greyMat;
  cvtColor(msg->image, greyMat, CV_BGR2GRAY);
  
  // Threshold filter: take binary image
  Mat img_bw = greyMat < 64;
  
  // When there's to much white, (e.g. original image is black), cancel operation
  if ( countNonZero(img_bw) > 1280*720/2 )
    return;
  
  // Construct skeleton image
  Mat skel(img_bw.size(), CV_8UC1, cv::Scalar(0));
  Mat temp;
  Mat eroded;
  
  Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
  bool done;
  do
  {
    erode(img_bw, eroded, element);
    dilate(eroded, temp, element);
    subtract(img_bw, temp, temp);
    bitwise_or(skel, temp, skel);
    eroded.copyTo(img_bw);
   
    // If (almost) all pixels in binary image are processed,
    // skeleton is good enough
    done = (countNonZero(img_bw) < 10);
  } while (!done);
  
  // Hough transformation to find lines
  vector<Vec4i> lines;
  vector<Vec4i> selected_lines;
  HoughLinesP(skel, lines, 1, CV_PI/180, 20, 10, 10);
  
  // SETTINGS for line selections
  // Big area definition (considered lines for all cases)
  const int big_minX = 1;
  const int big_maxX = 1279;
  const int big_minY = 1;
  const int big_maxY = 719;
  int lines_big_area_count = 0;
  
  // Small area defintion (considered lines for angle computations)
  const int small_minX = 640;
  const int small_maxX = 1150;
  const int small_minY = 150;
  const int small_maxY = 570;
  int lines_small_area_count = 0;
  
  // Variables to keep track of average angle in small area
  float A;
  vector<float> angles;
  
  // Variable to compute average y value in big area
  float y_values = 0.0;
  
  // Loop through lines
  for(size_t i = 0; i < lines.size(); i++)
  {
    // Only consider lines in big area
    if(lines[i][0] >= big_minX
      && lines[i][2] <= big_maxX
      && lines[i][1] >= big_minY
      && lines[i][3] <= big_maxY ) {
      
      // Compute line angle
      A = ( (float)lines[i][3] - (float)lines[i][1] )
      / ( (float)lines[i][2] - (float)lines[i][0] );
      
      // Only consider lines with a reasonable angle
      if (A > -10 && A < 10) {
        lines_big_area_count++;
        // Add y coordinate of line middle point to y_values
        y_values += (((float)lines[i][3] + (float)lines[i][1])/2);
        // Add line to big selection
        selected_lines.push_back(lines[i]);
        
        // Only add angle of line if it's in the small area
        if(lines[i][0] >= small_minX
          && lines[i][2] <= small_maxX
          && lines[i][1] >= small_minY
          && lines[i][3] <= small_maxY ) {
            lines_small_area_count++;
            // Add to small collection
            angles.push_back(A);
            // Color lines in small area blue
            line(img, Point(lines[i][0], lines[i][1]),
              Point(lines[i][2], lines[i][3]), Scalar(255,0,0), 3, 8);
        } else {
          // Color lines in big area red
          line(img, Point(lines[i][0], lines[i][1]),
            Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8);
        }
      }
    }
  }
  
  // If there are no lines in the big area, send no directions
  if(lines_big_area_count < 1) {
    return;
  }
  
  // Compute average y coordinate of line middle points
  float average_y = y_values / (float)lines_big_area_count;
    
  // Create Twist message
  geometry_msgs::Twist directions;
  directions.linear.x = 1;
  
  // Give directions based on corner cases:
  //   all lines are on the left side
  if(average_y < 150) {
    directions.angular.z = 1;
  }
  //   all lines are on the right side
  else if(average_y > 570) {
    directions.angular.z = -1;
  }
  // Give directions based on available lines:
  //   no lines in the small area
  else if(lines_small_area_count < 1) {
    // Average line on left side: send left
    if(average_y > 360) {
      directions.angular.z = 1;
    // Average line on right side: send right
    } else {
      directions.angular.z = -1;
    }
  }
  // Give directions based on lines in small area
  else {
    // Compute median of angles of all lines in small area
    sort(angles.begin(), angles.end());
    size_t size = angles.size();
    float median;
    if (size  % 2 == 0)
    {
      median = (angles[size / 2 - 1] + angles[size / 2]) / 2;
    }
    else 
    {
      median = angles[size / 2];
    }
    
    // Based on median, send robot in certain direction
    if (median > 0.2) {
      directions.angular.z = -1;
    } else if ( median < -0.2) {
      directions.angular.z = 1;
    } else {
      directions.angular.z = 0;
    }
  }
  
  // Show image with recognized lines 
  imshow(OPENCV_WINDOW, img);
  waitKey(3);
  
  // Publish directions
  pub.publish(directions);
}

// Main loop to listen to images on topic /camera/image
int main(int argc, char **argv)
{
  ros::init(argc, argv, "linefollower_21");
  ros::NodeHandle n;

  // Set buffer size to 1, to only use the most recent image
  ros::Subscriber sub = n.subscribe("camera/image", 1, imageCallback);
  pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  // ros::Rate loop_rate(10);

  ros::spin();

  return 0;
}

