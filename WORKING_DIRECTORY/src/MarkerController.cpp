
#include "MarkerController.hpp"
#include <iostream>


// Constructor
  MarkerController::MarkerController(STATE* state) : MissionController(state) {
      function_float_void.insert( std::make_pair( "Surge", &MarkerController::Surge ));
      function_float_void.insert( std::make_pair( "Sway", &MarkerController::Sway ));
      function_float_void.insert( std::make_pair( "Target", &MarkerController::Target ));
      function_float_void.insert( std::make_pair( "Align", &MarkerController::Align__modifier ));
  };

  void MarkerController::Task(std::string taskName)             // for "surge", "sway" and "target"
  {
    std::cout << "MarkerTask: beginning" << std::endl;
    double startTime = _state->global_time();

    while(_state->return_taskID() == 'm')       // 'm' for Marker Controller
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
    std::cout << "MarkerTask: centering of AUV complete" << std::endl;
  }


//---------------------------------------------------------------------------------

  void MarkerController::Task(std::string taskName_1, std::string taskName_2, float time)      // Task("Align", "Hover", time)    // perform taskName_1 and do taskName_2 @speed
  {

    std::cout << "MarkerTask: Bin alignment beginning" << std::endl;
    double startTime = _state->global_time();

    while(_state->return_taskID() == 'm')       // 'm' for MarkerTask
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
    
    std::cout << "MarkerTask: Bin alignment complete" << std::endl;
    std::cout << "MarkerTask: " << time << " second hover begun" << std::endl;

    double startTime_TASK2 = _state->global_time();
    while(_state->return_taskID() == 'm' && (_state->global_time() - startTime_TASK2) < time)
    {
      float theta_error = (this->*function_float_void[taskName_1])();
      hoverDepth(); // ADD timeout here so that it is not indefinite

      _state->thruster_update();
      usleep( 10000 );
      taskRunning();
    }
    taskComplete();
    if ((_state->global_time() - startTime_TASK2) >= time)
      std::cout << "MarkerTask: " << time << " second hover has been succesfully complete" << std::endl;
    else
      std::cout << "MarkerTask: Interrupted due to task switch" << std::endl;
  }

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------

    float MarkerController::Surge()  // vertical motion to reach target y-coordinate
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


    float MarkerController::Sway()  // sideways motion to reach the xError = 0 condition
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

    float MarkerController::yawAlign__modifier()
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


    float MarkerController::Align__modifier()
    {
      hoverPitch__modifier();
      yawAlign__modifier();
      return _state->return_phi();    
    }

    float MarkerController::Target()
    {
      Surge();
      Sway();
      return _state->return_r();
    }