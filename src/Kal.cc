/*
 * Kal.cc
 *
 *  Created on: 16 cze 2014
 *      Author: kglensk
 */
#include "Kal.hh"

Kalman roll_kalman;
Kalman pitch_kalman;


extern "C" double get_roll_angle_kalman(double  newAngle, double newRate, double dt)
{
	volatile double res = 0.0;
	res =  roll_kalman.getAngle(newAngle, newRate, dt );
  return res;
}

extern "C" void set_roll_angle_kalman(double newAngle)
{
  roll_kalman.setAngle(newAngle);
}

extern "C" double get_pitch_angle_kalman(double  newAngle, double newRate, double dtime)
{

  return pitch_kalman.getAngle(newAngle, newRate, dtime );
}

extern "C" void set_pitch_angle_kalman(double newAngle)
{
  pitch_kalman.setAngle(newAngle);
}

extern "C" void set_roll_qangle(double newQ_angle)
{
	roll_kalman.setQangle(newQ_angle);
}

extern "C" void set_roll_qbias(double newQ_bias)
{
	roll_kalman.setQbias(newQ_bias);
}


extern "C" void set_roll_rmeasure(double newR_measure)
{
	roll_kalman.setRmeasure(newR_measure);
}

extern "C" void roll_kalman_reset()
{
	roll_kalman.resetKalman();
}
