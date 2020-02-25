/**
 * @file GateDetector.cpp
 * @author Swapneel Naphade (naphadeswapneel@gmail.com)
 * @brief GateDetector class definition
 * @version 0.1
 * @date 01-05-2020
 * 
 *  Copyright (c) 2020 Swapneel Naphade
 * 
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 * 
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 * 
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

#include "racing_drone/GateDetector.hpp"
#include "common.cpp"

/**
 * @brief Calculate mean of pixel values in region of interest of an image
 * 
 * @param contour Vector of 4 points which make the contour for which ROI mean is calculated
 * @param image The input image 
 * @return double Mean of pixel values in the bounding box of the input contour on the image
 */
double roiMean(vector<Point> contour, Mat image)
{
	return mean(image(boundingRect(contour)))[0];
}

/**
 * @brief Compare areas of two contours, A and B
 * 
 * @param a Contour A
 * @param b Contour B
 * @return bool Is Area of A > Area of B ?
 */
bool compareContourArea(vector<Point> a, vector<Point> b)
{
        return (contourArea(a) > contourArea(b));
}

/**
 * @brief Calcualte aspect ratio of contour
 * 
 * @param contour Input contour
 * @return double Aspect Ratio of the contour (width-to-height ratio)
 */
double aspectRatio(const vector<Point>& contour)
{
	Rect boundRect( boundingRect(contour));
	return (double) (boundRect.height / boundRect.width);
}

/**
 * @brief Construct a new Gate Detector object
 * 
 * @param gateSide_ Side of the Square gate in meters
 * @param hsv_low_thresh_ HSV low threashold of the gate 
 * @param hsv_high_thresh_ HSV High threshold of the gate
 * @param blur_kernel_ Kernel size for bluring image
 * @param area_thresh_ Area threshold of the gate 
 * @param aspect_ratio_low_ Aspect Ratio low threshold of the gate (between 0.01 and 1)
 * @param roi_mean_thresh_ ROI Mean threshold
 * @param cameraMatrix_ Camera Matrix of camera
 * @param distCoeffs_ Distortion Coefficients of camera
 */
GateDetector::GateDetector( double gateSide_,
							Scalar hsv_low_thresh_,
							Scalar hsv_high_thresh_,
							int blur_kernel_,
							double area_thresh_,
							double aspect_ratio_low_,
							double roi_mean_thresh_,
							double gate_dist_thresh_,
							Mat cameraMatrix_,
							Mat distCoeffs_       )
	: 	gateSide(gateSide_),
		hsv_low_thresh(hsv_low_thresh_),
		hsv_high_thresh(hsv_high_thresh_),
		blur_kernel(blur_kernel_),
		area_thresh(area_thresh_),
		aspect_ratio_low(aspect_ratio_low_),
		roi_mean_thresh(roi_mean_thresh_),
		gate_dist_thresh(gate_dist_thresh_),
		cameraMatrix(cameraMatrix_),
		distCoeffs(distCoeffs_ )
{
	vector<Point> zero_gate{ Point(0.0), Point(0.0), Point(0.0), Point(0.0) };
	gateContour = zero_gate;
}     

/**
 * @brief Destroy the Gate Detector:: Gate Detector object
 * 
 */
GateDetector::~GateDetector(){}

/**
 * @brief Detect gate in the input image
 * 
 * @param image Input image for gate detection
 * @return true If gate is successfully detected
 * @return false If gate is not detected
 */
bool GateDetector::detectGate(Mat& image){
	
    // clock_t start_time = clock();
    // static int frame_count;
    // frame_count++;
     
	// ROS_INFO( "before bgr2hsv " );
    // Mat hsv, mask, blur;
	//Convert to HSV
	cvtColor(image, image, COLOR_BGR2HSV);
	//imshow("HSV", image);

	// ROS_INFO( "before mask " );
	//Mask
	inRange(image, hsv_low_thresh, hsv_high_thresh, image);
	// imshow("MASK", image);
	
	//Blur 
	GaussianBlur(image, image, Size(blur_kernel, blur_kernel), 5, 5);
	// imshow("BLUR", image);

	// ROS_INFO( "before find contours " );
	//Find contours
    //vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
	findContours( image, detectedContours, hierarchy, CV_RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
	// ROS_INFO("Number of contours in image: %d", detectedContours.size());

    vector<vector<Point>> allContours;
	for (uint i=0; i<detectedContours.size(); i++){
		vector<Point> approx;
        approxPolyDP(detectedContours[i], approx, 0.01f * arcLength(detectedContours[i], true), true);
		if (approx.size() == 4){
			allContours.push_back(approx);
        }
    }
	// ROS_INFO("Number of contours after polyApprox: %d", allContours.size());
    
	for(auto itr = allContours.begin(); itr != allContours.end(); )
	{
		if ( contourArea(*itr) < area_thresh )
			allContours.erase( itr );
		else
			itr++;
	}
	// ROS_INFO("Number of contours after Area thresh: %d", allContours.size());
	
    if (aspect_ratio_low > 1.0 || aspect_ratio_low < 0.05)
        aspect_ratio_low = 0.2;

	// Filter by contour aspect ratio: 1.20 > AR > 0.8
	for(auto itr = allContours.begin(); itr != allContours.end(); )
	{
		if( aspectRatio(*itr) <= aspect_ratio_low || aspectRatio(*itr) >= 1.0/aspect_ratio_low )	
			allContours.erase( itr );
		else
			itr++;
	}
	// ROS_INFO("Number of contours after aspect ratio: %d", allContours.size());

	// Filter by ROI mean: ROImean < 50
	for(auto itr = allContours.begin(); itr != allContours.end(); )
	{
		if( roiMean(*itr, image) > roi_mean_thresh )	
			allContours.erase( itr );
		else
			itr++;
	}
	// ROS_INFO("Number of contours after ROI mean: %d\n\n", allContours.size());

	for(int i=0; i < allContours.size(); i++)
	// std::cout << frame_count << "[" << i << "]: " << contourArea(allContours[i]) << std::endl; 

	vector<Point> sorted_gate(4);
	if( allContours.size() > 0 )
	{
		// ROS_INFO( "Contours found." );
		//Sort contours by area
    	sort( allContours.begin(), allContours.end(), compareContourArea );
	}
	else
	{
		// ROS_INFO( "Contours not found. Sending ZERO contour" );
		//Send 0 contour
		vector<Point> zero_gate{ Point(0.0), Point(0.0), Point(0.0), Point(0.0) };
		gateContour = zero_gate;
        return false;
	} 
	
	//Sort the points starting with top left and going counter-clockwise
	Point center;
	for(int i=0; i<4; i++)
	{
		center.x += allContours[0][i].x;
		center.y += allContours[0][i].y;
	}

	center.x = center.x / 4;
	center.y = center.y / 4;

	// ROS_INFO(" before contour point sorting. ");
	for(int i=0; i<4; i++)
	{
		if ( allContours[0][i].x <  center.x && allContours[0][i].y < center.y)
			gateContour[0] = allContours[0][i];
		else if ( allContours[0][i].x <  center.x && allContours[0][i].y >= center.y)
			gateContour[1] = allContours[0][i];
		else if ( allContours[0][i].x >=  center.x && allContours[0][i].y >= center.y)
			gateContour[2] = allContours[0][i];
		else
			gateContour[3] = allContours[0][i];
	}
				
    //Average Loop time without GPU:  0.042 s.
    // ROS_INFO( "[%d] Gate Detector Loop time: %.0f ms.", frame_count, ((float)(clock() - start_time))/CLOCKS_PER_SEC*1000.0 );
    return true;
} 

/**
 * @brief Calculate gate pose  with respect to drone
 * 
 * @return true If gate pose is successfully calculated 
 * @return false If gate pose is not calculated correctly
 */
bool GateDetector::getGatePose(void)
{
	//Camera matrix and distortionb coefficients
	// Mat cameraMatrix =  ( Mat_<double>(3,3) << 258.58131479, 0.0,  348.1852167,  0.0, 257.25344992, 219.07752178, 0.0, 0.0, 1.0) ;
	// Mat distCoeffs =  ( Mat_<double>(1,5) << -0.36310169, 0.10981468, 0.0057042, -0.001884,  -0.01328491) ;

	//Camera to drone rotation matrix
	Mat dRc = ( Mat_<double>(3,3) << 0,0,1,-1,0,0,0,-1,0 ) ;

	//Object Points
	vector<Point3d> objectPoints({
            Point3d(-gateSide/2.0, gateSide/2.0, 0.0),
			Point3d(-gateSide/2.0, -gateSide/2.0, 0.0),
			Point3d(gateSide/2.0, -gateSide/2.0, 0.0),
			Point3d(gateSide/2.0, gateSide/2.0, 0.0)});

	vector<Point2d> gateControur2d;
	for(auto itr=gateContour.begin(); itr != gateContour.end(); itr++ )
	{
		Point2d point;
		point.x = itr->x;
		point.y = itr->y;
		gateControur2d.push_back(point);
	}

	if ( solvePnP(objectPoints, gateControur2d, cameraMatrix, distCoeffs, rvec, tvec) )
	{
		rvec[0] += CV_PI;
		// ROS_INFO("rvec [cam]: %f, %f, %f", rvec[0], rvec[1], rvec[2]);
		//Convert rotation vector to euler angles
		std::vector<double> rpy(3);
		quat2rpy(rvec2quat(rvec), rpy);
		rvec = rpy;
		// ROS_INFO("rpy [cam]: %f, %f, %f", rpy[0], rpy[1], rpy[2]);

		// Vectors to matrices
		Mat Tvec = Mat(tvec, true).reshape(1,3);
		Mat Rvec = Mat(rvec, true).reshape(1,3);

		Tvec = dRc*Tvec;
		Rvec = dRc*Rvec;
		Tvec.col(0).copyTo(tvec);
		Rvec.col(0).copyTo(rvec);
		// ROS_INFO("rpy [drone]: %f, %f, %f", rvec[0], rvec[1], rvec[2]);

		// Gate distance check
		double gate_distance = sqrt( tvec[0]*tvec[0] + tvec[1]*tvec[1] + tvec[2]*tvec[2] );
		if( gate_distance > gate_dist_thresh)
			return false;
		else
        	return true;
	}

	else
	{
		fill(tvec.begin(), tvec.end(), 0.0);
		fill(rvec.begin(), rvec.end(), 0.0);
		return false;
	}
}