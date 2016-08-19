// FastStepper.cpp
//
// Copyright (C) 2009-2013 Mike McCauley
// $Id: FastStepper.cpp,v 1.21 2015/08/25 04:57:29 mikem Exp mikem $

#include "FastStepper.h"

void FastStepper::moveTo(long absolute)
{
    _targetPos = absolute;
    computeNewSpeed();
}

// Run the motor to implement speed and acceleration in order to proceed to the target position
// You must call this at least once per step, preferably in your main loop
// If the motor is in the desired position, the cost is very small
void FastStepper::run()
{
    if (runSpeed())
        computeNewSpeed();
}

FastStepper::FastStepper(void (*forward)(), void (*backward)(), float maxSpeed, float acceleration)
{
    _currentPos = 0;
    _targetPos = 0;
    _speed = 0.0;
    _maxSpeed = 1.0;
    _acceleration = 0.0;
    _inv_twoa = 1.0;
    _sqrt_twoa = 1.0;
    _stepInterval = 0;
    _lastStepTime = 0;
    _forward = forward;
    _backward = backward;

    // NEW
    _n = 0;
    _c0 = 0.0;
    _cn = 0.0;
    _cmin = 1.0;
    _direction = DIRECTION_CCW;

    // Some reasonable default
    setMaxSpeed(maxSpeed);
    setAcceleration(acceleration);

    _cn = _c0;
    while (_cn >= _cmin) {
      if (_n != 0) {
          // Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
          _cn = _cn - ((2.0 * _cn) / ((4.0 * _n) + 1)); // Equation 13
      }
      _pre_cn[_n] = (int) _cn;
      _n++;
    }
    _pre_nMaxSpeed = _n;
    _n = 0;
    _cn = 0.0;
}

void FastStepper::print() {
  for (int i = 0; i < 500; i++) {
    Serial.println(i);
    Serial.println(_pre_cn[i]);
  }
}

void FastStepper::setMaxSpeed(float speed)
{
    if (_maxSpeed != speed)
    {
        _maxSpeed = speed;
        _cmin = 1000000.0 / speed;
        // Recompute _n from current speed and adjust speed if accelerating or cruising
        if (_n > 0)
        {
            _n = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16
            computeNewSpeed();
        }
    }
}

void FastStepper::setAcceleration(float acceleration)
{
    if (acceleration == 0.0)
        return;
    if (_acceleration != acceleration)
    {
        // Recompute _n per Equation 17
        _n = _n * (_acceleration / acceleration);
        // New c0 per Equation 7, with correction per Equation 15
        _c0 = 0.676 * sqrt(2.0 / acceleration) * 1000000.0; // Equation 15
        _acceleration = acceleration;
        _inv_twoa = 1.0 / (2.0 * acceleration);
        computeNewSpeed();
    }
}


long FastStepper::distanceToGo()
{
    return _targetPos - _currentPos;
}


// Implements steps according to the current step interval
// You must call this at least once per step
// returns true if a step occurred
boolean FastStepper::runSpeed()
{
    // Dont do anything unless we actually have a step interval
    if (!_stepInterval)
        return false;

    unsigned long nextStepTime = _lastStepTime + _stepInterval;

    if ( micros() >= nextStepTime ) // wrap problem after 70 minutes
    {
        if (_direction == DIRECTION_CW)
        {
            // Clockwise
            _currentPos += 1;
            _forward();

        }
        else
        {
            // Anticlockwise
            _currentPos -= 1;
            _backward();
        }
        _lastStepTime = micros();
        return true;
    }
    else
    {
        return false;
    }
}

void FastStepper::computeNewSpeed()
{
    long distanceTo = _targetPos - _currentPos; // +ve is clockwise from curent location

    long stepsToStop = _n; //min(abs(_n), _pre_nMaxSpeed);

    if (distanceTo == 0 && stepsToStop <= 1) {
        // We are at the target and its time to stop
        _stepInterval = 0;
        _n = 0;
        return;
    }

    if (distanceTo > 0) {
        // We are anticlockwise from the target
        // Need to go clockwise from here, maybe decelerate now
        if (_pre_accelerating)
        {
            // Currently accelerating, need to decel now? Or maybe going the wrong way?
            if ((stepsToStop >= distanceTo) || _direction == DIRECTION_CCW)
              _pre_accelerating = false;
        }
        else if (!_pre_accelerating)
        {
            // Currently decelerating, need to accel again?
            if ((stepsToStop < distanceTo) && _direction == DIRECTION_CW)
                _pre_accelerating = true;
        }
    }
    else if (distanceTo < 0)
    {
        // We are clockwise from the target
        // Need to go anticlockwise from here, maybe decelerate
        if (_pre_accelerating)
        {
            // Currently accelerating, need to decel now? Or maybe going the wrong way?
            if ((stepsToStop >= -distanceTo) || _direction == DIRECTION_CW)
                _pre_accelerating = false; // Start deceleration
        }
        else if (!_pre_accelerating)
        {
            // Currently decelerating, need to accel again?
            if ((stepsToStop < -distanceTo) && _direction == DIRECTION_CCW)
                _pre_accelerating = true; // Start accceleration
        }
    }
    // Need to accelerate or decelerate
    if (_n == 0) {
        // First step from stopped
        _cn = _c0;
        _direction = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW;
        _n++;
    }
    else if (_n == _pre_nMaxSpeed) {
      _cn = _cmin;
      if (!_pre_accelerating) {
        _n--;
      }
    }
    else {
        _cn = _pre_cn[abs(_n)];

        if (_pre_accelerating) {
          _n++;
        } else {
          _n--;
        }
    }

    // Serial.println(distanceToGo());
    // Serial.println(_cn);
    // delay(200);

    _stepInterval = _cn; // implicit cast
    //_speed = 1000000.0 / _cn;
    // if (_direction == DIRECTION_CCW)
    //     _speed = -_speed;
}
