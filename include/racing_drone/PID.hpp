/**
 * PID.hpp
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
