/**
 * PID.cpp
 * 
 * Copyright 2019 Swapneel Naphade <naphadeswapneel@gmail.com>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
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
