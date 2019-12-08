#include "racing_drone/GateDetector.hpp"

double roiMean(vector<Point> contour, Mat image)
{
	return mean(image(boundingRect(contour)))[0];
}


bool compareContourArea(vector<Point> a, vector<Point> b){
        return (contourArea(a) > contourArea(b));
}

double aspectRatio(const vector<Point>& contour)
{
	Rect boundRect( boundingRect(contour));
	return (double) (boundRect.width / boundRect.height);
}


// Convert rotation vector to quaternion	
// def rvec2quat(vector):
// quat = [x,y,z,w]
vector<double> rvec2quat(vector<double> rvec)
{
	double theta=0;
	for(int i=0; i<rvec.size(); i++)
		theta += rvec[i]*rvec[i];

	theta = sqrt(theta);

	for(int i=0; i<rvec.size(); i++)
		rvec[i] /= theta;

	vector<double> quat(4);
	quat[0] = rvec[0] * sin(theta/2.0);
	quat[1] = rvec[1] * sin(theta/2.0);
	quat[2] = rvec[2] * sin(theta/2.0);
	quat[3] = cos(theta/2.0);

	return quat;
}

// Convert quaternion to euler angles (angles in radians)
// def quat2euler(q):
// 	q = [x,y,z,w]
// 	euler = [roll,pitch,yaw]
vector<double> quat2euler(vector<double> q)
{
	vector<double> rpy(3);
	rpy[0] = atan2( 2*(q[3]*q[0] + q[1]*q[2]), 1 - 2*(q[0]*q[0] + q[1]*q[1]) );
	rpy[1] = asin( 2*(q[3]*q[1] - q[0]*q[2]) );
	rpy[2] = atan2( 2*(q[3]*q[2] + q[0]*q[1]), 1 - 2*(q[1]*q[1]+ q[2]*q[2]) );
	return rpy;
}

// GateDetector Class definition
GateDetector::GateDetector( double gateSide_,
							Scalar hsv_low_thresh_,
							Scalar hsv_high_thresh_,
							int blur_kernel_,
							double area_thresh_,
							double aspect_ratio_low_,
							double roi_mean_thresh_,
							Mat cameraMatrix_,
							Mat distCoeffs_       )
	: 	gateSide(gateSide_),
		hsv_low_thresh(hsv_low_thresh_),
		hsv_high_thresh(hsv_high_thresh_),
		blur_kernel(blur_kernel_),
		area_thresh(area_thresh_),
		aspect_ratio_low(aspect_ratio_low_),
		roi_mean_thresh(roi_mean_thresh_),
		cameraMatrix(cameraMatrix_),
		distCoeffs(distCoeffs_ )
{

}     

GateDetector::~GateDetector(){}


bool GateDetector::detectGate(Mat& image){
	
    clock_t start_time = clock();
    static int frame_count;
    frame_count++;
     
    // Mat hsv, mask, blur;
	//Convert to HSV
	cvtColor(image, image, COLOR_BGR2HSV);
	//imshow("HSV", hsv);
	
	//Mask
	inRange(image, hsv_low_thresh, hsv_high_thresh, image);
	// imshow("MASK", mask);
	
	//Blur 
	GaussianBlur(image, image, Size(blur_kernel, blur_kernel), 5, 5);
	//imshow("BLUR", blur);

	//Find contours
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
	findContours( image, contours, hierarchy, CV_RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );

    vector<vector<Point>> allContours;
	for (uint i=0; i<contours.size(); i++){
		vector<Point> approx;
        approxPolyDP(contours[i], approx, 0.01f * arcLength(contours[i], true), true);
		if (approx.size() == 4){
			allContours.push_back(approx);
        }
    }
    
	for(auto itr = allContours.begin(); itr != allContours.end(); )
	{
		if ( contourArea(*itr) < area_thresh )
			allContours.erase( itr );
		else
			itr++;
	}
	
    if (aspect_ratio_low > 1.0 || aspect_ratio_low < 0.05)
        aspect_ratio_low = 0.5;

	// Filter by contour aspect ratio: 1.20 > AR > 0.8
	for(auto itr = allContours.begin(); itr != allContours.end(); )
	{
		if( aspectRatio(*itr) <= aspect_ratio_low || aspectRatio(*itr) >= 1.0/aspect_ratio_low )	
			allContours.erase( itr );
		else
			itr++;
	}

	// Filter by ROI mean: ROImean < 50
	for(auto itr = allContours.begin(); itr != allContours.end(); )
	{
		if( roiMean(*itr, image) > roi_mean_thresh )	
			allContours.erase( itr );
		else
			itr++;
	}

	for(int i=0; i < allContours.size(); i++)
	cout << frame_count << "[" << i << "]: " << contourArea(allContours[i]) << endl; 

	vector<Point> sorted_gate(4);
	if( allContours.size() > 0 )
	{
		//Sort contours by area
    	sort( allContours.begin(), allContours.end(), compareContourArea );
	}
	else
	{
		//Send 0 contour
		vector<Point> zero_gate{ Point(0.0), Point(0.0), Point(0.0), Point(0.0) };
		gateContour = zero_gate;
        return false;
	} 
	
	//Sort the points starting with top left and going anti-clockwise
	Point center;
	for(int i=0; i<4; i++)
	{
		center.x += allContours[0][i].x;
		center.y += allContours[0][i].y;
	}

	center.x = center.x / 4;
	center.y = center.y / 4;

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
    std::cout << "[" << frame_count << "]" << " Loop time: " << ((float)(clock() - start_time))/CLOCKS_PER_SEC*1000.0 << " ms." << std::endl;
    return true;
} 


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
		
		//Convert rotation vector to euler angles
		rvec = quat2euler(rvec2quat(rvec));

		// Vectors to matrices
		Mat Tvec = Mat(tvec, true).reshape(1,3);
		Mat Rvec = Mat(rvec, true).reshape(1,3);

		Tvec = dRc*Tvec;
		Rvec = dRc*Rvec;
		Tvec.col(0).copyTo(tvec);
		Rvec.col(0).copyTo(rvec);
        return true;
	}

	else
	{
		fill(tvec.begin(), tvec.end(), 0.0);
		fill(rvec.begin(), rvec.end(), 0.0);
		return false;
	}
}