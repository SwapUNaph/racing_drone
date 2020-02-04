/**
 * @file PID.hpp
 * @author Swapneel Naphade (naphadeswapneel@gmail.com)
 * @brief PID class declaration
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

#pragma once
#include <cmath>

class PID{ 
private:
	float P;
	float I;
	float D;
	float dt;
	float error;
	float integration;
	float derivative;
	float integration_limit;
public: 
	PID(float p, float i, float d, float dt_);
	~PID();
	float update(float err);
	void reset();
	float getP();
	float getI();
	float getD();
	float get_dt();
	void setP(float p);
	void setI(float i);
	void setD(float d);	
	void set_dt(float dt_);	
	void setIntLimit(float intLimit);	
};
