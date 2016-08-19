// FastStepper.h

#ifndef FastStepper_h
#define FastStepper_h

#include <stdlib.h>
#include <Arduino.h>


class FastStepper
{
public:
    FastStepper(void (*forward)(), void (*backward)(), float maxSpeed, float acceleration);

    /// Set the target position. The run() function will try to move the motor (at most one step per call)
    /// from the current position to the target position set by the most
    /// recent call to this function. Caution: moveTo() also recalculates the speed for the next step.
    /// If you are trying to use constant speed movements, you should call setSpeed() after calling moveTo().
    /// \param[in] absolute The desired absolute position. Negative is
    /// anticlockwise from the 0 position.
    void    moveTo(long absolute);

    /// Poll the motor and step it if a step is due, implementing
    /// accelerations and decelerations to acheive the target position. You must call this as
    /// frequently as possible, but at least once per minimum step time interval,
    /// preferably in your main loop. Note that each call to run() will make at most one step, and then only when a step is due,
    /// based on the current speed and the time since the last step.
    void run();

    /// The distance from the current position to the target position.
      /// \return the distance from the current position to the target position
      /// in steps. Positive is clockwise from the current position.
    long    distanceToGo();

    void print();

protected:

    /// \brief Direction indicator
    /// Symbolic names for the direction the motor is turning
    typedef enum
    {
	      DIRECTION_CCW = 0,  ///< Clockwise
        DIRECTION_CW  = 1   ///< Counter-Clockwise
    } Direction;

    /// Forces the library to compute a new instantaneous speed and set that as
    /// the current speed. It is called by
    /// the library:
    /// \li  after each step
    /// \li  after change to maxSpeed through setMaxSpeed()
    /// \li  after change to acceleration through setAcceleration()
    /// \li  after change to target position (relative or absolute) through
    /// move() or moveTo()
    void           computeNewSpeed();

    // information
    boolean runSpeed();

    /// Sets the maximum permitted speed. The run() function will accelerate
    /// up to the speed set by this function.
    /// Caution: the maximum speed achievable depends on your processor and clock speed.
    /// \param[in] speed The desired maximum speed in steps per second. Must
    /// be > 0. Caution: Speeds that exceed the maximum speed supported by the processor may
    /// Result in non-linear accelerations and decelerations.
    void    setMaxSpeed(float speed);

    /// Sets the acceleration/deceleration rate.
    /// \param[in] acceleration The desired acceleration in steps per second
    /// per second. Must be > 0.0. This is an expensive call since it requires a square
    /// root to be calculated. Dont call more ofthen than needed
    void    setAcceleration(float acceleration);


private:

    /// The current absolution position in steps.
    long           _currentPos;    // Steps

    /// The target position in steps. The FastStepper library will move the
    /// motor from the _currentPos to the _targetPos, taking into account the
    /// max speed, acceleration and deceleration
    long           _targetPos;     // Steps

    /// The current motos speed in steps per second
    /// Positive is clockwise
    float          _speed;         // Steps per second

    /// The maximum permitted speed in steps per second. Must be > 0.
    float          _maxSpeed;

    /// The acceleration to use to accelerate or decelerate the motor in steps
    /// per second per second. Must be > 0
    float          _acceleration;
    float          _inv_twoa; // Precomputed 1/(2*a)
    float          _sqrt_twoa; // Precomputed sqrt(2*_acceleration)

    /// The current interval between steps in microseconds.
    /// 0 means the motor is currently stopped with _speed == 0
    unsigned long  _stepInterval;

    /// The last step time in microseconds
    unsigned long  _lastStepTime;

    /// The pointer to a forward-step procedure
    void (*_forward)();

    /// The pointer to a backward-step procedure
    void (*_backward)();

    /// The step counter for speed calculations
    long _n;

    /// Initial step size in microseconds
    float _c0;

    /// Last step size in microseconds
    float _cn;

    /// Min step size in microseconds based on maxSpeed
    float _cmin; // at max speed

    /// Current direction motor is spinning in
    boolean _direction; // 1 == CW

    // precalculated distance to stop
    int _pre_cn[500];
    int _pre_nMaxSpeed;
    boolean _pre_accelerating;
};

#endif
