
#ifndef _LANECONTROLLER
#define _LANECONTROLLER

#include "TYPEDEF.hpp"
#include "MissionController.hpp"

class LaneController : public MissionController
{

private:
  typedef float (LaneController::*fnPtr_float)(void);
  std::map <std::string, fnPtr_float> function_float_void;

// Constructor
public:
  LaneController(STATE* state);

public:

  void Task(std::string taskName);             // for "surge", "sway" and "target"

//---------------------------------------------------------------------------------

  void Task(std::string taskName_1, std::string taskName_2, int speed);      // Task("Align", "Surge", speed)    // perform taskName_1 and do taskName_2 @speed

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------

    float Surge();  // vertical motion to reach target y-coordinate

    float Sway();  // sideways motion to reach the xError = 0 condition
    
    float yawAlign__modifier();

    float Align__modifier();

    float Target();
};

#endif
