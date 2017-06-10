
#include "BasicController.hpp"

  /*
  static std::map<std::string, std::function<void(void)>> function_void_void;
  static std::map<std::string, std::function<void(int)>> function_void_int;
  */
     
// Constructor
  BasicController::BasicController(STATE* state) : MissionController(state) {
      function_void_void.insert( std::make_pair( "Hover", &BasicController::Hover ));
      function_void_int.insert( std::make_pair( "Sink", &BasicController::Sink ));
      function_void_int.insert( std::make_pair( "Surge", &BasicController::Surge ));
      function_void_int.insert( std::make_pair( "Sway", &BasicController::Sway ));
  };

//-----------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------- taskName \in {Hover, Surge, Sway, Sink} ----------------------------------------------

  void BasicController::timedTask(std::string taskName, double duration)    // "taskName" for "duration" seconds
  {
    std::cout << "BasicController: " << taskName << " beginning, for a duration of " << duration << std::endl;
    double startTime = _state->global_time();
    
    while(_state->global_time() - startTime <  duration) {
      (this->*function_void_void[taskName])();
      _state->thruster_update();
      usleep( 10000 );
      taskRunning();
    }
    taskComplete();
    std::cout << "task complete" << std::endl;
  }

  void BasicController::timedTask(std::string taskName, int speed, double duration)    // "taskName" at "speed" for "duration" seconds
  {
    std::cout << "BasicController: " << taskName << " beginning, for a duration of " << duration << ", at a speed of " << speed << std::endl;
    double startTime = _state->global_time();

    while(_state->global_time() - startTime <  duration) {
      (this->*function_void_int[taskName])(speed);
      _state->thruster_update();
      usleep( 10000 );    // Sleep for 0.01s to free up CPU resources
      taskRunning();
    }
    taskComplete();
    std::cout << "task complete" << std::endl;
  }

  void BasicController::conditionalTask(std::string taskName, char flag)    // "flag" gives us the task_ID condition till when the 
  {                                                                         // task is run
    std::cout << "BasicController: " << taskName << " beginning, while IP flag = " << flag << std::endl;
    double startTime = _state->global_time();

    while(_state->return_taskID() == flag)
    {
      (this->*function_void_void[taskName])();
      _state->thruster_update();
      usleep( 10000 );    // Sleep for 0.01s to free up CPU resources    
      taskRunning();
    }
    taskComplete();
    std::cout << "task complete" << std::endl;  
  }

  void BasicController::conditionalTask(std::string taskName, int speed, char flag)    // "flag" gives us the taskID till when the
  {                                                                                    // task is run
    std::cout << "BasicController: " << taskName << " beginning, at a speed of" << speed << " while IP flag = " << flag << std::endl;    
    double startTime = _state->global_time();
    
    while(_state->return_taskID() == flag)
    {
      (this->*function_void_int[taskName])(speed);
      _state->thruster_update();
      usleep( 10000 );    // Sleep for 0.01s to free up CPU resources
      taskRunning();
    }
    taskComplete();
    std::cout << "task complete" << std::endl;    
  }  

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------

  void BasicController::Hover()
  {
    resetPWM();
    hoverDepth();
    deadAlign__modifier();
  }

  void BasicController::Surge(int speed)
  {
    deadSurge(speed);
    deadAlign__modifier();
  }

  void BasicController::Sway(int speed)
  {
    deadSway(speed);
    deadAlign__modifier();
  }

  void BasicController::Sink(int speed)
  {
    deadSink(speed);
    deadAlign__modifier();
  }
