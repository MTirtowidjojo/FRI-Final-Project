#include "person_detector.cpp"

/**
 * Kicks off person detection by creating a person detector
 * calling ros::spin() to continuously read in info
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "opencv_person_detector");
    PersonDetector pd;
    ros::spin();
    return 0;
}
