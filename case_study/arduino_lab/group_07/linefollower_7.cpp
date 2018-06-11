#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";
static const unsigned int MINIMUM_LINE_WIDTH = 10;
static const unsigned int MAXIMUM_LINE_WIDTH = 200;
static const unsigned int IMAGE_SIZE_X = 600;
static const unsigned int IMAGE_SIZE_Y = 800;
static const int THRESHOLD_VALUE = 63;
static const int ROI_X_OFFSET = 10;
static const int ROBOT_SPEED = 125;

//Structure that represents a line
typedef struct Line {
    int left;
    int right;
} Line;

class ImageConverter {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher cmd_vel_pub_;
    geometry_msgs::Twist message;

public:
    ImageConverter()
            : it_(nh_) {
        // Subscribe to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/camera/image", 1,
                                   &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);


        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter() {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat rotation;
        cv::Mat standard;
        cv::Mat gray;
        cv::Mat blur;
        cv::Mat threshold;
        cv::Mat noise;

        // Rotate the image 90 degrees
        cv::transpose(cv_ptr->image, rotation);
        cv::flip(rotation, rotation, 1);

        // Resize the image
        cv::resize(rotation, standard, cv::Size(IMAGE_SIZE_X, IMAGE_SIZE_Y), 0, 0, 1);

        /*
         * Check if image has been received.
         */
        if (rotation.cols > 60) {
            // Define Region of interest
            cv::Rect rectangle(ROI_X_OFFSET, 6 * standard.rows / 8, standard.cols - 2 * ROI_X_OFFSET, standard.rows / 8);
            cv::Mat region(standard, rectangle);

            // Make region gray
            cv::cvtColor(region, gray, CV_BGR2GRAY);

            // Blur
            cv::GaussianBlur(gray, blur, cv::Size(9, 9), 0, 0, 0);

            // Threshold
            cv::threshold(blur, threshold, THRESHOLD_VALUE, 255, cv::THRESH_BINARY);

            // Remove any noise left
            int morphSize = 2;
            cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * morphSize + 1, 2 * morphSize + 1),
                                                        cv::Point(morphSize, morphSize));
            cv::morphologyEx(threshold, noise, cv::MORPH_CLOSE, element, cv::Point(-1, -1), 1);

            /*
             * Find all the lines and select the best one.
             */
            std::vector <Line> lineVector;
            int lineWidth = 0;
            int beginLine = 0;

            for (int i = 0; i < noise.cols; i++) {
                int sumOfCol = 0;
                for (int j = 0; j < noise.rows; j++) {
                    if (noise.at<u_char>(j, i)) {
                        sumOfCol++;
                    }
                }
                // When white
                if (sumOfCol > noise.rows / 2) {
                    if (lineWidth > MINIMUM_LINE_WIDTH && lineWidth < MAXIMUM_LINE_WIDTH) {
                        Line line;
                        line.left = beginLine;
                        line.right = i - 1;
                        lineVector.push_back(line);
                    }
                    lineWidth = 0;
                }
                // When black
                else {
                    if (lineWidth == 0) {
                        beginLine = i;
                    }
                    lineWidth++;
                }
            }

            // Check if there might be a line that does not contain a transition back to white.
            if (lineWidth > MINIMUM_LINE_WIDTH && lineWidth < MAXIMUM_LINE_WIDTH) {
                Line line;
                line.left = beginLine;
                line.right = noise.cols - 1;
                lineVector.push_back(line);
            }

            int bestLine = 0;
            std::vector<Line>::iterator v = lineVector.begin();

            // Select the best line out of all lines found.
            while (v != lineVector.end()) {
                cv::line(standard, cv::Point(ROI_X_OFFSET + v->left, 6 * standard.rows / 8),
                         cv::Point(ROI_X_OFFSET + v->left, 7 * standard.rows / 8), CV_RGB(255, 0, 0));
                cv::line(standard, cv::Point(ROI_X_OFFSET + v->right, 6 * standard.rows / 8),
                         cv::Point(ROI_X_OFFSET + v->right, 7 * standard.rows / 8), CV_RGB(255, 0, 0));
                int middle = (v->left + v->right) / 2;
                if (std::abs(middle - noise.cols / 2) < std::abs(bestLine - noise.cols / 2)) {
                    bestLine = middle;
                }
                v++;
            }

            // When a line has been found, display the best line on screen and publish a twist message for it.
            if (bestLine) {
                cv::line(standard, cv::Point(ROI_X_OFFSET + bestLine, 6 * standard.rows / 8),
                         cv::Point(ROI_X_OFFSET + bestLine, 7 * standard.rows / 8), CV_RGB(0, 0, 255));
                message.linear.x = ROBOT_SPEED;
                message.angular.z = (bestLine - noise.cols / 2) * 90 / (noise.cols / 2);
                cmd_vel_pub_.publish(message);
            }

            // Draw region of interest on screen with green.
            cv::rectangle(standard, rectangle, CV_RGB(0, 255, 0));
        }

        // Update GUI Window
        cv::imshow(OPENCV_WINDOW, standard);
        cv::waitKey(10);

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}
