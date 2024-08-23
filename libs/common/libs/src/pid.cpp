/**
 * Copyright 2019 Bradley J. Snyder <snyder.bradleyj@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include "pid.h"

using namespace std;

class PIDImpl
{
	public:
		PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki );
		~PIDImpl();
		double calculate( double setpoint, double pv );
		double calculate_error( double error );
		void set_kp(double Kp);
		void set_kd(double Kd);
		void set_ki(double Ki);
		double get_error();

	private:
		double _dt;
		double _max;
		double _min;
		double _Kp;
		double _Kd;
		double _Ki;
		double _pre_error;
		double _integral;
};


PID::PID( double dt, double max, double min, double Kp, double Kd, double Ki )
{
	pimpl = new PIDImpl(dt,max,min,Kp,Kd,Ki);
}

void PID::set_kp(double Kp)
{
	pimpl->set_kp(Kp);
}

void PID::set_kd(double Kd)
{
	pimpl->set_kd(Kd);
}

void PID::set_ki(double Ki)
{
	pimpl->set_ki(Ki);
}

double PID::calculate( double setpoint, double pv )
{
	return pimpl->calculate(setpoint,pv);
}

double PID::calculate_error( double error)
{
	return pimpl->calculate_error(error);
}

double PID::get_error()
{
	return pimpl->get_error();
}

PID::~PID()
{
	delete pimpl;
}

/**
 * Implementation
 */
PIDImpl::PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki ):
	_dt(dt),
	_max(max),
	_min(min),
	_Kp(Kp),
	_Kd(Kd),
	_Ki(Ki),
	_pre_error(0),
	_integral(0)
{
}

double
PIDImpl::
calculate( double setpoint, double pv )
{

	// Calculate error
	double error = setpoint - pv;

	// Proportional term
	double Pout = _Kp * error;

	// Integral term
	_integral += error * _dt;
	// if(fabs(error) > 50) _integral = 0;
	double Iout = _Ki * _integral;

	// Derivative term
	double derivative = (error - _pre_error) / _dt;
	double Dout = _Kd * derivative;
	// Calculate total output
	double output = Pout + Iout + Dout;
	// printf("Pout: %02lf, Iout %02lf, Dout %02lf, out %02lf\n", Pout, Iout, Dout, output);

	// Restrict to max/min
	if( output > _max )
		output = _max;
	else if( output < _min )
		output = _min;

	// Save error to previous error
	_pre_error = error;

	return output;
}

double
PIDImpl::
calculate_error( double error)
{
	// Proportional term
	double Pout = _Kp * error;

	// Integral term
	_integral += error * _dt;
	double Iout = _Ki * _integral;

	// Derivative term
	double derivative = (error - _pre_error) / _dt;
	double Dout = _Kd * derivative;

	// Calculate total output
	double output = Pout + Iout + Dout;

	// Restrict to max/min
	if( output > _max )
		output = _max;
	else if( output < _min )
		output = _min;

	// Save error to previous error
	_pre_error = error;

	// printf("output: %f, error %f, integral %f \n", output, error, _integral);

	int offset = 10;
	output = (output > 0) ? (output + offset) : (output - offset);
	return output;
}

void PIDImpl::set_kp(double Kp)
{
	_Kp = Kp;
}

void PIDImpl::set_kd(double Kd)
{
	_Kd = Kd;
}

void PIDImpl::set_ki(double Ki)
{
	_Ki = Ki;
}

double PIDImpl::get_error()
{
	return _pre_error;
}

PIDImpl::~PIDImpl()
{
}

#endif
