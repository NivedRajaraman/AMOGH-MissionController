
#include "LaneController.hpp"
#include <iostream>


// Constructor
  LaneController::LaneController(STATE* state) : MissionController(state) {
      function_float_void.insert( std::make_pair( "Surge", &LaneController::Surge ));
      function_float_void.insert( std::make_pair( "Sway", &LaneController::Sway ));
      function_float_void.insert( std::make_pair( "Target", &LaneController::Target ));
      function_float_void.insert( std::make_pair( "Align", &LaneController::Align__modifier ));
  };

  void LaneController::Task(std::string taskName)             // for "surge", "sway" and "target"
  {
    std::cout << "LaneTask: " << taskName << " task beginning"<< std::endl;
    double startTime = _state->global_time();

    while(_state->return_taskID() == 'p')       // 'p' for PathTask (LaneTask)
    {
      float error = (this->*function_float_void[taskName])();
      deadAlign__modifier();
      _state->thruster_update();
      usleep( 10000 );
      taskRunning();

      if(error < PIXERR)    // add condition to break out of loop due to timeout
        break;
    }
    taskComplete();
    std::cout << "LaneTask: " << taskName << " task complete"<< std::endl;
  }


//---------------------------------------------------------------------------------

  void LaneController::Task(std::string taskName_1, std::string taskName_2, int speed)      // Task("Align", "Surge", speed)    // perform taskName_1 and do taskName_2 @speed
  {
    std::cout << "LaneTask: Lane alignment beginning"<< std::endl;
    double startTime = _state->global_time();

    while(_state->return_taskID() == 'p')       // 'p' for PathTask (LaneTask)
    {
      float theta_error = (this->*function_float_void[taskName_1])();
      _state->thruster_update();
      usleep( 10000 );

      if (theta_error < ANGERR)
        break;
      taskRunning();
    }
    taskComplete();    
    resetPWM();

    std::cout << "LaneTask: Alignment task complete"<< std::endl;
    std::cout << "LaneTask: Surge beginning at a speed = " << speed << std::endl;

    double startTime_TASK2 = _state->global_time();
    while(_state->return_taskID() == 'p')
    {
      float theta_error = (this->*function_float_void[taskName_1])();
      deadSurge(speed); // ADD timeout here so that it is not indefinite surge
      _state->thruster_update();
      usleep( 10000 );
      taskRunning();
    }
    taskComplete();
    std::cout << "LaneTask: Task complete. Next task will now begin" << std::endl;
  }

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------

    float LaneController::Surge()  // vertical motion to reach target y-coordinate
    {
      static float Kp = 0.05;
      static float Ki = 0.02;

      static float yIntegralError = 0;

      if(taskStatus())
        yIntegralError = 0;

      float yError = (_state->return_r())*sin(_state->return_theta());
      yIntegralError = yIntegralError + yError*(_state->return_timeDiff());   // updates @ IP updation rate
      
      int speed = Kp*yError + Ki*yIntegralError;

      deadSurge(speed);

      return yError;
    }


    float LaneController::Sway()  // sideways motion to reach the xError = 0 condition
    {
      static float Kp = 0.05;
      static float Ki = 0.02;

      static float xIntegralError = 0;

      if(taskStatus())
        xIntegralError = 0;

      float xError = (_state->return_r())*cos(_state->return_theta());
      
      xIntegralError = xIntegralError + xError*(_state->return_timeDiff());
      
      int speed = Kp*xError + Ki*xIntegralError;

      deadSway(speed);

      return xError;
    }

    float LaneController::yawAlign__modifier()
    {
      static float Kp = 0.05;
      static float Ki = 0.02;

      static float phiIntegralError = 0;
      int PWM[2];
      
      if(taskStatus())
        phiIntegralError = 0;

      float phiError = _state->return_phi();
      phiIntegralError = phiIntegralError + phiError*(_state->return_timeDiff());
      
      int PWM_Back_modifier = Kp*phiError + Ki*phiIntegralError;
    
      _state->return_PWM_Back(PWM);
      PWM[0] = PWM[0] + PWM_Back_modifier;
      PWM[1] = PWM[1] - PWM_Back_modifier;
      _state->set_PWM_Back(PWM);

      return phiError;
    }


    float LaneController::Align__modifier()
    {
      hoverPitch__modifier();
      yawAlign__modifier();
      return _state->return_phi();    
    }

    float LaneController::Target()
    {
      Surge();
      Sway();
      return _state->return_r();
    }