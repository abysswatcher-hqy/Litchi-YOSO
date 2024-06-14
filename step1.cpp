#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <set>

using namespace cv;
using namespace std;

// Function to apply skeleton extraction
Mat applySkeleton(const Mat &src) {
    Mat skel(src.size(), CV_8UC1, Scalar(0));
    Mat temp, eroded;
    Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));

    do {
        erode(src, eroded, element);
        dilate(eroded, temp, element); // temp = open(src)
        subtract(src, temp, temp);
        bitwise_or(skel, temp, skel);
        eroded.copyTo(src);

    } while (countNonZero(src) != 0);

    return skel;
}

// Function to perform erosion operation
Mat erodeImage(const Mat &src) {
    Mat eroded;
    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    erode(src, eroded, element);
    return eroded;
}

// Function to find bounding rectangle of the skeleton
Rect findBoundingRect(const Mat &skel) {
    vector<vector<Point>> contours;
    findContours(skel, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    return boundingRect(contours[0]);
}

// Function to create sparse matrix from skeleton
vector<Point> createSparseMatrix(const Mat &skel) {
    vector<Point> points;
    for (int i = 0; i < skel.rows; i++) {
        for (int j = 0; j < skel.cols; j++) {
            if (skel.at<uchar>(i, j) == 255) {
                points.push_back(Point(j, i));
            }
        }
    }
    return points;
}

// Function to get the connectivity matrix for a given point
vector<int> getConnectivityMatrix(const Mat &skel, Point pt) {
    vector<int> connectivity(8, 0);
    int directions[8][2] = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};

    for (int i = 0; i < 8; i++) {
        int newX = pt.x + directions[i][0];
        int newY = pt.y + directions[i][1];
        if (newX >= 0 && newX < skel.cols && newY >= 0 && newY < skel.rows) {
            if (skel.at<uchar>(newY, newX) == 255) {
                connectivity[i] = 1;
            }
        }
    }
    return connectivity;
}

// Function to find boundary points based on connectivity
vector<Point> findBoundaryPoints(const Mat &skel, const vector<Point> &points) {
    vector<Point> boundaryPoints;

    for (const auto &pt : points) {
        vector<int> connectivity = getConnectivityMatrix(skel, pt);
        int sum = accumulate(connectivity.begin(), connectivity.end(), 0);
        if (sum <= 3) {
            boundaryPoints.push_back(pt);
        }
    }

    // Filter points to keep only max index points for contiguous indices
    set<Point> filteredBoundaryPoints;
    for (size_t i = 0; i < boundaryPoints.size(); i++) {
        if (i == boundaryPoints.size() - 1 || boundaryPoints[i + 1].x != boundaryPoints[i].x + 1 || boundaryPoints[i + 1].y != boundaryPoints[i].y + 1) {
            filteredBoundaryPoints.insert(boundaryPoints[i]);
        }
    }

    return vector<Point>(filteredBoundaryPoints.begin(), filteredBoundaryPoints.end());
}

// Function to build point-line model
vector<pair<Point, Point>> buildPointLineModel(const vector<Point> &boundaryPoints, const Mat &skel) {
    vector<pair<Point, Point>> pointLineModel;

    for (size_t i = 0; i < boundaryPoints.size(); i++) {
        for (size_t j = i + 1; j < boundaryPoints.size(); j++) {
            vector<int> connectivityA = getConnectivityMatrix(skel, boundaryPoints[i]);
            vector<int> connectivityB = getConnectivityMatrix(skel, boundaryPoints[j]);

            if (connectivityA[0] == 1 && connectivityB[7] == 1) {
                pointLineModel.emplace_back(boundaryPoints[i], boundaryPoints[j]);
            }
        }
    }

    return pointLineModel;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lychee_branch_extraction");
    ros::NodeHandle nh;

    // Load segmented image (assume image is already segmented)
    Mat segmented = imread("segmented_image.png", IMREAD_GRAYSCALE);
    if (segmented.empty()) {
        ROS_ERROR("Failed to load image");
        return -1;
    }

    // Apply skeleton extraction
    Mat skeleton = applySkeleton(segmented);

    // Perform erosion to remove noise and burrs
    Mat erodedSkeleton = erodeImage(skeleton);

    // Find bounding rectangle of the skeleton
    Rect boundingRect = findBoundingRect(erodedSkeleton);

    // Create sparse matrix
    vector<Point> sparseMatrix = createSparseMatrix(erodedSkeleton);

    // Find boundary points
    vector<Point> boundaryPoints = findBoundaryPoints(erodedSkeleton, sparseMatrix);

    // Build point-line model
    vector<pair<Point, Point>> pointLineModel = buildPointLineModel(boundaryPoints, erodedSkeleton);

    // Print the point-line model
    for (const auto &line : pointLineModel) {
        ROS_INFO("Line from (%d, %d) to (%d, %d)", line.first.x, line.first.y, line.second.x, line.second.y);
    }

    ros::spin();
    return 0;
}
