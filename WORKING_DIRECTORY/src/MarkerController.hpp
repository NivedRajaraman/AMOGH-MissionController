
#ifndef _MARKERCONTROLLER
#define _MARKERCONTROLLER

#include "TYPEDEF.hpp"
#include "MissionController.hpp"

class MarkerController : public MissionController
{

private:
  typedef float (MarkerController::*fnPtr_float)(void);
  std::map <std::string, fnPtr_float> function_float_void;

// Constructor
public:
  MarkerController(STATE* state);

public:

  void Task(std::string taskName);             // for "surge", "sway" and "target"

//---------------------------------------------------------------------------------

  void Task(std::string taskName_1, std::string taskName_2, float time);      // Task("Align", "Hover", time)    // perform taskName_1 and then do taskName_2 for (time)

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------

    float Surge();  // vertical motion to reach target y-coordinate

    float Sway();  // sideways motion to reach the xError = 0 condition
    
    float yawAlign__modifier();

    float Align__modifier();

    float Target();
};

#endif
