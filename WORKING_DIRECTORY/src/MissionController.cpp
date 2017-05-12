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


//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

#include "MissionController.hpp"

MissionController::MissionController(STATE* state){
	this->_state = state;
};
  
// taskComplete is set if a task is done and helps initialize the next task.
// Significant to reset the accumulated integral error. An internal function
// running in a while loop keeps track of the accumulated error over its 
// repeated calls (through a static variable). When the same function is called
// later on in a different task, we need to be able to identify that the accumulated
// error should now start from 0 and not take over from its old value. taskComplete
// helps solve this.

// Calling this function signifies that the current task is complete :: CHECK FOR FALSE POSITIVE RESPONSE
void MissionController::taskComplete() {
	this->RESET = true;
}

// Gives the task status
// True means that task has just completed and new task can start
// False means that the task is in the process of runnning
bool MissionController::taskStatus() {
	return this->RESET;
}

// Complement to taskComplete()
void MissionController::taskRunning() {
	this->RESET = false;
} 

// FIGURE OUT A WAY TO ESTIMATE DEPTHS THROUGH JUST THE CAMERA
// "translation" function - tries to do the above. Look at the help doc to understand what it does
float MissionController::Inv(float x) {
	return 1.0/(1.0+x);
}

//-------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------ ROTATION CONTROLLERS ---------------------------------------------------

// modifies current PWM values such that the PITCH does not change from the original value from when the function is first called
// Comes with its own PI controller for rotational pitch control
// MODIFIES THE BOTTOM THRUSTER PWM values (differential PWM about bias)
float MissionController::hoverPitch__modifier()
{
	static float Kp = 0.05;
	static float Ki = 0.0;
	static float pitchIntegralError = 0;
	static float HoverPitch;
	int PWM[2];

	if(taskStatus() == true)
	{
		HoverPitch = _state->return_pitch();
		pitchIntegralError = 0;
	}
    	
	float pitchError = HoverPitch - _state->return_pitch();
	pitchIntegralError = pitchError*(_state->return_timeDiff());

	int PWM_Bottom_modifier = Kp*pitchError + Ki*pitchIntegralError;

	_state->return_PWM_Bottom(PWM);
	PWM[0] = PWM[0] + PWM_Bottom_modifier;
	PWM[1] = PWM[1] - PWM_Bottom_modifier; 
	_state->set_PWM_Bottom(PWM);

	return pitchError;
}

// modifies current PWM values such that the YAW does not change from the original value from when the function is first called
// Comes with its own PI controller for in-plane yaw control
// MODIFIES THE BACK THRUSTER PWM values (differential PWM about bias)
float MissionController::hoverYaw__modifier()
{
	static float Kp = 0.05;
	static float Ki = 0.0;
	static float yawIntegralError = 0;
	static float HoverYaw;
	int PWM[2];

	if(taskStatus() == true)
	{
		HoverYaw = _state->return_yaw();
		yawIntegralError = 0;
	}

	float yawError = HoverYaw - _state->return_yaw();
	yawIntegralError = yawIntegralError + yawError*(_state->return_timeDiff());

	// account for deadband of 25 usec in PWM :: refer to AMOGH/ArduinoCodeV1/PWM_I2C_specs.png
	int PWM_Back_modifier = Kp*yawError + Ki*yawIntegralError;

	_state->return_PWM_Back(PWM);
	PWM[0] = PWM[0] - PWM_Back_modifier;
	PWM[1] = PWM[1] + PWM_Back_modifier;
	_state->set_PWM_Back(PWM);

	return yawError;
}

//-------------------------------------------------------------------------------------------------------------------------------

// sets bottom PWM value such that the DEPTH does not change from the original value from when the function is first called
// Comes with its own PI controller for depth control
// MODIFIES THE ALL THRUSTER VALUES (as it calls hoverPitch__modifier() and hoverYaw__modifier(), and controls the depth itself)
float MissionController::hoverDepth()
{
	static float Kp = 0.05;
	static float Ki = 0.0;

	static float pressureIntegralError = 0;
	static float HoverPressure;

	if(taskStatus() == true)
	{
		HoverPressure = _state->return_pressure();
		pressureIntegralError = 0;
	}

	float pressureError = HoverPressure - _state->return_pressure();
	pressureIntegralError = pressureIntegralError + pressureError*(_state->return_timeDiff());

	// account for deadband of 25 usec in PWM :: refer to AMOGH/ArduinoCodeV1/PWM_I2C_specs.png
	int PWM_Bottom = Kp*pressureError + Ki*pressureIntegralError + 1500 + BuoyancyConstant;
	
	// BuoyancyConstant corresponds to the Buoyancy of the AUV 
	// which is the steady state PWM value (SSE under pure P motion is always 0 
	// which can't sustain the AUV, with integral control, it can be phased out)
	
	int PWM[2] = {PWM_Bottom, PWM_Bottom};
	_state->set_PWM_Bottom(PWM);

	return pressureError;
}

// resets PWM values to 1500. no shit.
void MissionController::resetPWM() {
	int PWM[2] = {1500, 1500};
	_state->set_PWM_Bottom(PWM);
	_state->set_PWM_Back(PWM);
	_state->set_PWM_Side(PWM);
}

//-------------------------------------------------------------------------------------------------------------------------------------

// SINK with @param "speed" as downward PWM velocity. No PID on lateral motion, it can sway/surge/rotate naturally with water currents
// NO SPEED FEEDBACK :: does not ensure that @param "speed" has been achieved
void MissionController::deadSink(int speed)
{
	int PWM[2] = {BuoyancyConstant + speed + 1500, BuoyancyConstant + speed + 1500};
	_state->set_PWM_Bottom(PWM);
}

// SURGE with @param "speed" as forward velocity. No PID on sway/rotation. PID on depth.
// NO SPEED FEEDBACK :: does not ensure that @param "speed" has been achieved
void MissionController::deadSurge(int speed)
{
	hoverDepth();
	int PWM[2] = {1500 + speed, 1500 + speed};
	_state->set_PWM_Back(PWM);
}

// SWAY with @param "speed" as sideway velocity. No PID on surge/rotation. PID on depth.
// NO SPEED FEEDBACK :: does not ensure that @param "speed" has been achieved
void MissionController::deadSway(int speed)	
{
	hoverDepth();	
	int PWM[2] = {1500 + speed, 1500 + speed};
	_state->set_PWM_Side(PWM);
}

// Since we use hoverYaw__modifier() and hoverPitch__modifier() together a lot,
void MissionController::deadAlign__modifier()
{
	hoverYaw__modifier();
	hoverPitch__modifier();
}

// init static variable, RESET
bool MissionController::RESET = false;
