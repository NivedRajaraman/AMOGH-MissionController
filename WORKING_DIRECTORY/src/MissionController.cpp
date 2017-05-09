/* Functioning:
   
  main()
  {
    HAS A BASIC THRUSTER CONTROL OBJECT CALLED MISSIONCONTROLLER THAT IS THE DEFAULT RESPONSE OF THE AUV IN WATER (NOT SURE WHAT IT IS
    SUPPOSED TO DO AS IT IS SWITCHED ON AND PUT INTO THE WATER). THE IP KEEPS CHECKING THE SURROUNDINGS. IF IT SEES A BUOY, BUOYTASK *INTITIATES*.
    BUOYTASK IS A EXTENDED SUBCLASS OF MISSIONCONTROLLER TASK. THEY SHARE SOME COMMON STATIC VARIABLES
    THE MEANING OF INITIATE IS:
    AS THE AUV IS TURNED ON, OBJECTS OF DIFFRENT CONTROL TYPES ARE INTIATED (EG. FOR RED BUOY BUOYTASK, LANETASK etc.) THE DATA WHICH THE 
    NEW OBJECT WILL USE TO CONTROL IT IS STORED IN THE STATIC VARIABLES WHICH IS INHERENTLY PASSED ON. 
   }
*/ 

// IMU roll -> -180 to +180 :: clockwise roll is positive
// IMU pitch -> -90 to +90 :: positive pitch is nose up
// IMU yaw -> 0 to 360 :: top-view-clockwise is positive


//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

#include "MissionController.hpp"

  MissionController::MissionController(STATE* state){
  	this->_state = state;
  };

  void MissionController::taskComplete() {      // taskComplete is set if a task is done and helps initialize the next task
    this->RESET = true;
  }

  bool MissionController::taskStatus() {					// True means that task has just completed and new task can start
  	return this->RESET;				// False means that the task is in the process of runnning
  }

  void MissionController::taskRunning() {
    this->RESET = false;
  } 

  float MissionController::Inv(float x) {
    return 1.0/(1.0+x);
  }

//-----------------------------------------------------------------------------------------------------------------------------------------------------

   // modifies current PWM values such that the PITCH does not change from the original value from when the function is first called

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


  // sets bottom PWM value such that the DEPTH does not change from the original value from when the function is first called
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
	  int PWM_Bottom = Kp*pressureError + Ki*pressureIntegralError + 1500 + BuoyancyConstant;		// BuoyancyConstant corresponsds to the Buoyancy of the AUV 
		   																																												// which is the steady state PWM value (SSE under pure P motion is always 0 
		   																																												// which can't sustain the AUV)
	  int PWM[2] = {PWM_Bottom, PWM_Bottom};
	  _state->set_PWM_Bottom(PWM);

	  return pressureError;
	}

	void MissionController::resetPWM() {
		int PWM[2] = {1500, 1500};
		_state->set_PWM_Bottom(PWM);
		_state->set_PWM_Back(PWM);
		_state->set_PWM_Side(PWM);
	}

//-----------------------------------------------------------------------------------------------------------------------------------------------------

	void MissionController::deadSink(int speed)			// SINK with @param "speed" as downward PWM velocity. No PID on lateral motion, it can sway/surge/rotate naturally.
	{															// NO SPEED FEEDBACK :: does not ensure that @param "speed" has been achieved
		int PWM[2] = {BuoyancyConstant + speed + 1500, BuoyancyConstant + speed + 1500};
		_state->set_PWM_Bottom(PWM);
	}

	void MissionController::deadSurge(int speed)			// SURGE with @param "speed" as forward velocity. No PID on sway/rotation. PID on depth.
	{															// NO SPEED FEEDBACK :: does not ensure that @param "speed" has been achieved
		hoverDepth();
		int PWM[2] = {1500 + speed, 1500 + speed};
		_state->set_PWM_Back(PWM);
	}

	void MissionController::deadSway(int speed)			// SWAY with @param "speed" as sideway velocity. No PID on surge/rotation. PID on depth.
	{															// NO SPEED FEEDBACK :: does not ensure that @param "speed" has been achieved
		hoverDepth();	
		int PWM[2] = {1500 + speed, 1500 + speed};
		//std::cout << PWM[0] << std::endl;
		_state->set_PWM_Side(PWM);
	}

	void MissionController::deadAlign__modifier()
	{
		hoverYaw__modifier();
		hoverPitch__modifier();
	}
//-----------------------------------------------------------------------------------------------------------------------------------------------------

bool MissionController::RESET = false;