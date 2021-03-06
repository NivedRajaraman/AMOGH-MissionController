/* functioning:
   
    The basic thruster control object called MissionController details the basic functions that control the motion of the AUV.
    The MissionController class contains the basic feedbacked (Eg. hovering: requires constant comparison with initial location)
    and feedbackless (Eg. back thrusters @ 1750 PWM: the AUV may sway due to underwater currents but that is irrelevant here)
    tasks. 
*/

// NOTE:
// IMU roll -> -180 to +180 :: clockwise roll is positive
// IMU pitch -> -90 to +90 :: positive pitch is nose up
// IMU yaw -> 0 to 360 :: top-view-clockwise is positive

#ifndef _MISSION_CONTROLLER
#define _MISSION_CONTROLLER

#define BuoyancyConstant 0

#include "state.hpp"
#include <cmath>
#include "TYPEDEF.hpp"

// IMU roll -> -180 to +180 :: clockwise roll is positive
// IMU pitch -> -90 to +90 :: positive pitch is nose up
// IMU yaw -> 0 to 360 :: top-view-clockwise is positive


//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

class MissionController
{

// Constructor
public:
  MissionController(STATE* state);
  STATE* _state;

protected:
	// RESET resets the static variables in the corresponding action function when we move to some other task and come back
	// For example, if you "Hover" some time, the integral error terms would have some non-zero value in it. When we perform
	// "forward" for a while and come back to Hover, the integral error term should be reinitialized to 0 again, hence, RESET
	static bool RESET;

private:
	float Kp;
	float Ki;

protected:

	void taskComplete();

	// taskStatus returns the task status (no shit, sherlock)
	// True means that task has just completed and new task can start
	// False means that the task is in the process of runnning
	bool taskStatus();

	void taskRunning();

	// "Translation function" refer to help doc for more info
	float Inv(float x);

//-----------------------------------------------------------------------------------------------------------------------------------------------------

public:

	//modifies current PWM values such that the PITCH does not change from the original value from when the function is first called
	float hoverPitch__modifier();

	// modifies current PWM values such that the YAW does not change from the original value from when the function is first called
	float hoverYaw__modifier();

	// sets bottom PWM value such that the DEPTH does not change from the original value from when the function is first called
	float hoverDepth();

	void resetPWM();

//-----------------------------------------------------------------------------------------------------------------------------------------------------

	void deadSink(int speed);			// SINK with @param "speed" as downward PWM velocity. No PID on lateral motion, it can sway/surge/rotate naturally.

	void deadSurge(int speed);			// SURGE with @param "speed" as forward velocity. No PID on sway/rotation. PID on depth.

	void deadSway(int speed);			// SWAY with @param "speed" as sideway velocity. No PID on surge/rotation. PID on depth.

	void deadAlign__modifier();

//-----------------------------------------------------------------------------------------------------------------------------------------------------
};

#endif
