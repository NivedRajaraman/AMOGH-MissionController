
#include "BuoyController.hpp"
#include "TYPEDEF.hpp"


// Constructor
  BuoyController::BuoyController(STATE* state) : MissionController(state) {
    function_float_void.insert( std::make_pair( "Sink", &BuoyController::Sink ));
    function_float_void.insert( std::make_pair( "Surge", &BuoyController::Surge ));
    function_float_void.insert( std::make_pair( "Sway", &BuoyController::Sway ));
    function_float_void.insert( std::make_pair( "Target", &BuoyController::Target ));
  };

  void BuoyController::Task(std::string taskName)       // NEED to modify for forward motion buoy collision detection
  {
    std::cout << "BuoyTask: " << taskName << " beginning" << std::endl;
    double startTime = _state->global_time();

    while(_state->return_taskID() == 'b')
    {
      float error = (this->*function_float_void[taskName])();
      deadAlign__modifier();
      _state->thruster_update();
      usleep( 10000 );
      taskRunning();

      if(error < PIXERR)        // add condition to break out of loop due to timeout
        break;
    }
    taskComplete();
    std::cout << "BuoyTask: " << taskName << " complete" << std::endl;  
  }
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------

    float BuoyController::Sink()  // vertical motion to reach target depth
    {
      static float Kp = 0.05;
      static float Ki = 0.02;

      static float yIntegralError;
      float yError, speed;

      if(taskStatus() == true)
        yIntegralError = 0;

      yError = (_state->return_r())*sin(_state->return_theta());
      yIntegralError = yIntegralError + yError*(_state->return_timeDiff());   // updates @ IP updation rate
      
      speed = Kp*yError + Ki*yIntegralError;

      deadSink(speed);
      return yError;
    }


    float BuoyController::Surge()
    {
      static float Kp = 0.05;
      static float Ki = 0.02;

      static float dIntegralError;
      float dError, speed;

      if(taskStatus() == true)
        dIntegralError = 0;

      dError = Inv(_state->return_r());
      dIntegralError = dIntegralError + dError*(_state->return_timeDiff());     // updates @ IP updation rate

      speed = Kp*dError + Ki*dIntegralError;

      deadSurge(speed);
      return dError;
    }



    float BuoyController::Sway()  // sideways motion to reach the xError = 0 condition
    {
      static float Kp = 0.05;
      static float Ki = 0.02;
      
      static float xIntegralError;
      float xError, speed;

      if(taskStatus() == true)
        xIntegralError = 0;

      xError = _state->return_r()*cos(_state->return_theta());
      xIntegralError = xIntegralError + xError*(_state->return_timeDiff());
      
      speed = Kp*xError + Ki*xIntegralError;

      deadSway(speed);
      return xError;
    }

    float BuoyController::Target()
    {
      Sway();
      Surge();
      Sink();
      return _state->return_r();
    }
