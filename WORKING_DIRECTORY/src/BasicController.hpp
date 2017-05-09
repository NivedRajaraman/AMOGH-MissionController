

#ifndef _BASICCONTROLLER
#define _BASICCONTROLLER

#include "TYPEDEF.hpp"
#include "MissionController.hpp"

class BasicController : public MissionController
{
private:
  /*
  static std::map<std::string, std::function<void(void)>> function_void_void;
  static std::map<std::string, std::function<void(int)>> function_void_int;
  */

  typedef void (BasicController::*fnPtr_void)(void);
  typedef void (BasicController::*fnPtr_int)(int);
  std::map <std::string, fnPtr_void> function_void_void;
  std::map <std::string, fnPtr_int> function_void_int;
     
// Constructor
public:
  BasicController(STATE* state);

public:
//-----------------------------------------

  void timedTask(std::string taskName, double duration);

  void timedTask(std::string taskName, int speed, double duration);    // "taskName" at "speed" for "duration" 

  void conditionalTask(std::string taskName, char flag);    // "flag" gives us the condition till when the task is run

  void conditionalTask(std::string taskName, int speed, char flag);    // "flag" gives us the condition till when the task is run 

//----------------------------------------

  void Hover();

  void Surge(int speed);

  void Sway(int speed);

  void Sink(int speed);
};


/*  std::map<std::string, std::function<void(void)>> BasicController::function_void_void = {"Hover", Hover};
  std::map<std::string, std::function<void(int)>> BasicController::function_void_int = {{"Sink", Sink},{"Surge",Surge},{"Sway", Sway}};*/

#endif
