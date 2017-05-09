
#ifndef _GATECONTROLLER
#define _GATECONTROLLER

#include "TYPEDEF.hpp"
#include "MissionController.hpp"

class GateController : public MissionController
{

private:
  typedef float (GateController::*fnPtr_float)(void);
  std::map <std::string, fnPtr_float> function_float_void;

// Constructor
public:
  GateController(STATE* state);

public:
  void Task(std::string taskName);
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------

  float Sink();

  float Surge();

  float Sway();

  float Target();
};

#endif