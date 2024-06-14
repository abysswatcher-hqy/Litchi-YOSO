#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace cv;

void detectCorners(const Mat &image, vector<Point> &corners) {
    Mat gray, cornersMat;
    cvtColor(image, gray, COLOR_BGR2GRAY);
    goodFeaturesToTrack(gray, corners, 100, 0.01, 10);

    for (const auto &corner : corners) {
        circle(image, corner, 5, Scalar(0, 0, 255), 2);
    }
}

pair<Point, int> findSmoothestRegion(const Mat &image, const vector<Point> &corners) {
    int windowWidth = image.cols / 10;
    int windowHeight = image.rows;
    int stepSize = image.cols / 20;
    int minCornerCount = INT_MAX;
    Point bestCenter;

    for (int x = 0; x <= image.cols - windowWidth; x += stepSize) {
        Rect window(x, 0, windowWidth, windowHeight);
        int cornerCount = 0;

        for (const auto &corner : corners) {
            if (window.contains(corner)) {
                cornerCount++;
            }
        }

        if (cornerCount < minCornerCount) {
            minCornerCount = cornerCount;
            bestCenter = Point(x + windowWidth / 2, windowHeight / 2);
        }
    }

    return {bestCenter, minCornerCount};
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lychee_picking_point_detection");
    ros::NodeHandle nh;

    // Load the segmented main branch image
    Mat mainBranchImage = imread("main_branch_image.png");
    if (mainBranchImage.empty()) {
        ROS_ERROR("Failed to load main branch image");
        return -1;
    }

    // Detect corners in the main branch image
    vector<Point> corners;
    detectCorners(mainBranchImage, corners);

    // Find the smoothest region and determine the picking point
    auto [pickingPoint, cornerCount] = findSmoothestRegion(mainBranchImage, corners);

    // Mark the picking point on the image
    circle(mainBranchImage, pickingPoint, 10, Scalar(0, 255, 0), 3);

    // Display the result
    namedWindow("Picking Point Detection", WINDOW_AUTOSIZE);
    imshow("Picking Point Detection", mainBranchImage);
    waitKey(0);

    // Output the picking point coordinates
    ROS_INFO("Picking point is at (%d, %d) with %d corners in the window", pickingPoint.x, pickingPoint.y, cornerCount);

    ros::spin();
    return 0;
}
