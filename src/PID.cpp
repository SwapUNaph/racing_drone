/**
 * @file PID.cpp
 * @author Swapneel Naphade (naphadeswapneel@gmail.com)
 * @brief PID class definiton
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

#include "racing_drone/PID.hpp"

/**
 * @brief PID class constructor
 * @param p Proportional Gain
 * @param i Intergrator Gain
 * @param d Derivative Gain
 * @param dt_ Time step
 * @returns nothing
 * 
 * 
 */
PID::PID(float p, float i, float d, float dt_) : P(p), I(i), D(d), dt(std::abs(dt_)){
	integration = 0;
	derivative = 0;
	error = 0;
	integration_limit = 5.0;
}

/**
 * @brief PID class destructor
 * @returns nothing
 * 
 * 
 */
PID::~PID(){}

/**
 * @brief Updates the PID controller output given the error in measurement and
 * 			desired value
 * @param err Desired value - Measurement value
 * @returns Control input
 * 
 * 
 */
float PID::update(float err){
	
	integration += (error + err) / 2 * dt;
	integration = (integration > integration_limit) ? integration_limit : integration;
	integration = (integration < -integration_limit) ? -integration_limit : integration;
	
	derivative = (error - err) / dt;
	error = err;
	
	return P * err + I * integration + D * derivative;
}

/**
 * @brief Resets integration, derivative and error to 0
 * 
 * 
 */
void PID::reset(){
	integration = 0;
	derivative = 0;
	error = 0;
}

/**
 * @brief Get P gain
 * @returns P
 * 
 * 
 */
float PID::getP(){
	return P;
}

/**
 * @brief Get I gain
 * @returns I
 * 
 * 
 */
float PID::getI(){
	return I;
}

/**
 * @brief Get D gain
 * @returns D
 * 
 * 
 */
float PID::getD(){
	return D;
}

/**
 * @brief 
 * @returns 
 * 
 * 
 */
float PID::get_dt(){
	return dt;
}	

/**
 * @brief Set P gain
 * @param p P gain for PID controller
 * 
 * 
 */
void PID::setP(float p){
	P = p;
}
/**
 * @brief Set I gain
 * @param i I gain for PID controller
 * 
 * 
 */
void PID::setI(float i){
	I = i;
}

/**
 * @brief Set D gain
 * @param d D gain for PID controller
 * 
 * 
 */
void PID::setD(float d){
	D = d;
}

/**
 * @brief Set time step
 * @param dt_ time step
 * 
 * 
 */
void PID::set_dt(float dt_){
	dt = std::abs(dt_);
}		

/**
 * @brief Set integration limit
 * @param intLimit Integration Limit for intergrator output
 * 
 * 
 */
void PID::setIntLimit(float intLimit){
	integration_limit = std::abs(intLimit);
}
