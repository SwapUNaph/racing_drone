/**
 * @file common.cpp
 * @author Swapneel Naphade (naphadeswapneel@gmail.com)
 * @brief Helper functions common to all the modules
 * @version 0.1
 * @date 01-06-2020
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


#include <vector>
#include <cmath>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>

namespace ublas = boost::numeric::ublas;


/**
 * @brief Convert rotation vector to quaternion
 * 
 * @param rvec Rotation vector -> length is the angle and axis of rotation is along the vector
 * @return Quaternion -> [x,y,z,w]
 */
std::vector<double> rvec2quat(std::vector<double> rvec)
{
	double theta = 0.0;
	for(int i=0; i<rvec.size(); i++)
		theta += rvec[i]*rvec[i];

	theta = std::sqrt(theta);

	for(int i=0; i<rvec.size(); i++)
		rvec[i] /= theta;

	std::vector<double> quat(4);
	quat[0] = rvec[0] * sin(theta/2.0);
	quat[1] = rvec[1] * sin(theta/2.0);
	quat[2] = rvec[2] * sin(theta/2.0);
	quat[3] = cos(theta/2.0);

	return quat;
}


/**
 * @brief Convert roll-pitch-yaw angles to quaternion
 * 
 * @param rpy rpy-vector [r,p,y] (in radians)
 * @param quat quaternion-vector [x,y,z,w]
 */
void rpy2quat(const std::vector<double>& rpy, std::vector<double>& quat)
{
	double cy = cos(rpy[2] * 0.5);
    double sy = sin(rpy[2] * 0.5);
    double cp = cos(rpy[1] * 0.5);
    double sp = sin(rpy[1] * 0.5);
    double cr = cos(rpy[0] * 0.5);
    double sr = sin(rpy[0] * 0.5);
    
    quat[0] = cy * cp * sr - sy * sp * cr;
    quat[1] = sy * cp * sr + cy * sp * cr;
    quat[2] = sy * cp * cr - cy * sp * sr;
	quat[3] = cy * cp * cr + sy * sp * sr;

	return;
}

/**
 * @brief Convert Quaternion to roll-pitch-yaw
 * 
 * @param q Quaternion [x,y,z,w]
 * @param rpy Roll-Pitch-Yaw [r,p,y] (in radians)
 */
void quat2rpy(const std::vector<double>& q, std::vector<double>& rpy )
{
	rpy[0] = atan2( 2*(q[3]*q[0] + q[1]*q[2]), 1 - 2*(q[0]*q[0] + q[1]*q[1]) );
	rpy[1] = asin( 2*(q[3]*q[1] - q[0]*q[2]) );
	rpy[2] = atan2( 2*(q[3]*q[2] + q[0]*q[1]), 1 - 2*(q[1]*q[1]+ q[2]*q[2]) );
	return;
}

/**
 * @brief Multiply two quaternions { q1 (x) q2 }
 * 
 * @param q1 Quaternion 1 [x,y,z,w]
 * @param q2 Quaternion 2 [x,y,z,w]
 * @return std::vector<double> Quaternion Product
 */
std::vector<double> quatMultiply(const std::vector<double>& q1, const std::vector<double>& q2)
{
    std::vector<double> qProd(4);
    qProd[3] = q1[3]*q2[3] - q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2]; //w
    qProd[0] = q1[3]*q2[0] + q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1]; //x
    qProd[1] = q1[3]*q2[1] - q1[0]*q2[2] + q1[1]*q2[3] + q1[2]*q2[0]; //y
    qProd[2] = q1[3]*q2[2] + q1[0]*q2[1] - q1[1]*q2[0] + q1[2]*q2[3]; //z

    return qProd;
}

/**
 * @brief Conjugate of Quaternion { q* }
 * 
 * @param q Quaterion [x,y,z,w]
 * @return std::vector<double> Quternion Conjugate
 */
std::vector<double> quatConjugate(const std::vector<double>& q)
{
    std::vector<double> qConj(4);
    qConj[3] =  q[3]; //w
    qConj[0] = -q[0]; //x
    qConj[1] = -q[1]; //y
    qConj[2] = -q[2]; //z

    return qConj;
}

/**
 * @brief Difference of Rotation Quaterion { q1 (x) q2* }
 * 
 * @param q1 Quaternion 1 [x,y,z,w]
 * @param q2 Quaternion 2 [x,y,z,w]
 * @return std::vector<double> Rotation Quaternion Difference 
 */
std::vector<double> quatDifference(const std::vector<double>& q1, const std::vector<double>& q2)
{
    std::vector<double> qDiff(4);
    qDiff = quatMultiply(q1, quatConjugate(q2));

    return qDiff;
}

/**
 * @brief Rotate ublas vector with quaternion
 * 
 * @param q Quaternion [x,y,z,w]
 * @param vec boost ublas vector 3
 * @return ublas::vector<double> Rotated boost ublas vector
 */
ublas::vector<double> rotateVec(std::vector<double> q, ublas::vector<double> vec)
{
    ublas::vector<double> result(3);
    std::vector<double> vecQ(4), rotVecQ(4);
    vecQ[0] = vec(0);
    vecQ[1] = vec(1);
    vecQ[2] = vec(2);
    vecQ[3] = 0.0;

    rotVecQ = quatMultiply(q, quatMultiply(vecQ, quatConjugate(q)));

    result(0) = rotVecQ[0];
    result(1) = rotVecQ[1];
    result(2) = rotVecQ[2];

    return result;
}


/**
 * @brief Rotate std vector with quaternion
 * 
 * @param q Quaternion [x,y,z,w]
 * @param vec std vector 3
 * @return std::vector<double> Rotated std vector
 */
std::vector<double> rotateVec(std::vector<double> q, std::vector<double> vec)
{
    std::vector<double> vecQ(4), rotVecQ(4), result(3);
    vecQ[0] = vec[0];
    vecQ[1] = vec[1];
    vecQ[2] = vec[2];
    vecQ[3] = 0.0;

    rotVecQ = quatMultiply(q, quatMultiply(vecQ, quatConjugate(q)));

    result[0] = rotVecQ[0];
    result[1] = rotVecQ[1];
    result[2] = rotVecQ[2];

    return result;
}