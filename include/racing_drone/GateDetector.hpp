#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <ctime>
#include <algorithm>
#include <vector>


using namespace cv;
using namespace std;

double roiMean(vector<Point> contour, Mat image);
bool compareContourArea(vector<Point> a, vector<Point> b);
double aspectRatio(const vector<Point>& contour);
vector<double> rvec2quat(vector<double> rvec);
vector<double> quat2euler(vector<double> q);

class GateDetector
{
    public:
    double gateSide;
    Scalar hsv_low_thresh;
    Scalar hsv_high_thresh;
    int blur_kernel;
    double area_thresh;
    double aspect_ratio_low;
    double roi_mean_thresh;
    Mat cameraMatrix;
    Mat distCoeffs;


    vector<Point> gateContour;
    vector<double> tvec;
    vector<double> rvec;

    GateDetector(   double gateSide_,
                    Scalar hsv_low_thresh_,
                    Scalar hsv_high_thresh_,
                    int blur_kernel_,
                    double area_thresh_,
                    double aspect_ratio_low_,
                    double roi_mean_thresh_,
                    Mat cameraMatrix_,
                    Mat distCoeffs_             );

    ~GateDetector();
    bool detectGate(Mat& image);
    bool getGatePose(void);

};