#include <string>
#include <vector>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>

static const std::string OUTPUT_WINDOW = "Output window";

/**
 * Class to perform person detection and publish the HumanArea of the detected person
 * Code based on People Detection Sample from OpenCV by Bryan Chung found at:
 * http://www.magicandlove.com/blog/2011/12/04/people-detection-sample-from-opencv/
 */
class PersonDetector {
    private:
        ros::NodeHandle n;
        ros::Publisher point_pub;
        ros::Publisher sound_pub;
        image_transport::ImageTransport it;
        image_transport::Subscriber image_sub;
        image_transport::Publisher image_pub;
        cv::HOGDescriptor hog;
        bool received_cb;
        geometry_msgs::Point image_center;

        void detect_person(cv::Mat&);
        void draw_and_publish(cv::Mat&, cv::Rect&);
        void image_cb(const sensor_msgs::ImageConstPtr&);

    public:
        PersonDetector();
        ~PersonDetector();

};
