#include "person_detector.h"
#include <opencv_person_detector/HumanArea.h>

using opencv_person_detector::HumanArea;
using geometry_msgs::Point;

typedef std::vector<cv::Rect>::const_iterator rect_it;

/**
 * Creates a new PersonDetector given the topic to subscribe to in order
 * to get the rgb image data
 */
PersonDetector::PersonDetector() : it(n) {
    sound_pub = n.advertise<std_msgs::Bool>("/detection_sound", 1);
    point_pub = n.advertise<HumanArea>("/opencv_person_detector/center_points", 1);
    image_sub = it.subscribe("/nav_kinect/rgb/image_color", 1, &PersonDetector::image_cb, this);
    cv::namedWindow(OUTPUT_WINDOW);
    hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
}

/**
 * PersonDetector destructor to destroy the opencv windows
 */
PersonDetector::~PersonDetector() {
    cv::destroyWindow(OUTPUT_WINDOW);
}

/**
 * Callback for the images to process the image and detect people
 * Checks for people detected and shows the video with the detected people
 */
void PersonDetector::image_cb(const sensor_msgs::ImageConstPtr& msg) {
    received_cb = true;
    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bdrige exception: %s", e.what());
        return;
    }

    // Getting center of the image and drawing center line
    cv::Mat output_img = cv_ptr->image.clone();
    image_center.x = output_img.cols / 2.0;
    image_center.y = output_img.rows / 2.0;
    cv::line(output_img, cv::Point(image_center.x, 0), cv::Point(image_center.x, output_img.rows), CV_RGB(255, 0, 0), 1);

    // Detecting people
    detect_person(output_img);

    // Showing results
    cv::imshow(OUTPUT_WINDOW, output_img);
    cv::waitKey(3);
    image_pub.publish(cv_ptr->toImageMsg());
}

/**
 * Performs the people detection given a cv::Mat of the image
 */
void PersonDetector::detect_person(cv::Mat& img) {
    std::vector<cv::Rect> found, filtered;
    hog.detectMultiScale(img, found, 0, cv::Size(8, 8), cv::Size(32, 32), 1.05, 2);
    std_msgs::Bool play_sound;
    play_sound.data = true;

    for (size_t i = 0; i < found.size(); i++) {
        cv::Rect rect = found[i];

        size_t j = 0;
        while (j < found.size() && !(j != i && (rect & found[j]) == rect)) {
            j++;
        }

        if (j == found.size()) {
                        // Only draw and publish if the height of the possible person is at least 300 pixels
            if (rect.height >= 300) {
                if (play_sound.data) {
                    sound_pub.publish(play_sound);
                    play_sound.data = false;
                }

                draw_and_publish(img, rect);
            }
        }
    }
}

/**
 * Draws a rectangle around the person and publishes the HumanArea
 */
void PersonDetector::draw_and_publish(cv::Mat& img, cv::Rect& rect) {
    // Drawing the rectangle
    rect.x += cvRound(rect.width * 0.1);
    rect.width = cvRound(rect.width * 0.8);
    rect.y += cvRound(rect.height * 0.06);
    rect.height = cvRound(rect.height * 0.9);
    cv::rectangle(img, rect.tl(), rect.br(), cv::Scalar(0, 255, 0), 2);


    // Publishing the HumanArea of the detected human
    HumanArea ha;
    ha.human.human_center.x = rect.x + (rect.width / 2.0);
    ha.human.human_center.y = rect.y + (rect.height / 2.0);
    ha.human.human_center.z = 0.0;
    ha.human.width = rect.width;
    ha.human.height = rect.height;
    ha.image_center = image_center;
    point_pub.publish(ha);
}

