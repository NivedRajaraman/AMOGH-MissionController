
#ifndef _STATE
#define _STATE 

#include <chrono>
#include <ctime>
#include <ratio>
#include <cmath>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

class STATE
{

protected:
  char _taskID;    // IP taskID :: H is Halt, B is buoyTask,
 
  // startTime :: time of beginning (wall_clock)
  // timeDiff (time between IMU updates :: needed for integral control)
  std::chrono::steady_clock::time_point _startTime;
  float _timeDiff;  //16th bit

  // IP (camera) readings
  float* _r;      
  float* _theta;  
  float* _phi;   

  // angular position measurments
  float _roll;
  float _pitch;
  float _yaw;

  // Gyro readings (angular velocity)
  float _gyroX;
  float _gyroY;
  float _gyroZ;

  // Pressure sensor readings
  float _pressure;

  // PWM values
  int _PWM_Side[2];
  int _PWM_Back[2];
  int _PWM_Bottom[2];

  int _Port;
public :
// Constructor
  STATE();
  //~STATE();
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&      FLOW CONTROL FUNCTIONS     &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/

/*Set and get the value of the flag (flow control variable)*/

  void set_taskID(char C);
  char return_taskID();

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&      ARDUINO PORT FUNCTIONS     &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/

  void set_port(int port);

  void thruster_update();


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&        DATA FUNCTIONS       &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/

// ---------------------------------------------------TIME FUNCTIONS-----------------------------------------------------

  double global_time();

  void set_timeDiff(float t);

  float return_timeDiff() ;

// -----------------------------------------------POSITION & IP FUNCTIONS-------------------------------------------------

// IP (camera) readings, r, theta and phi

  void set_r(float r);
  void set_theta(float theta);
  void set_phi(float phi);


  float return_r();
  float return_theta();
  float return_phi();

//-----------------------------------------------------------------------------------------------------------------------------
// Roll, pitch and yaw readings from the IMU

  void set_roll(float roll);
  void set_pitch(float pitch);
  void set_yaw(float yaw);


  float return_roll();
  float return_pitch();
  float return_yaw();;

//-----------------------------------------------------------------------------------------------------------------------------
// gyroX gyroY gyroZ readings

  void set_gyroX(float gyroX);
  void set_gyroY(float gyroY);
  void set_gyroZ(float gyroZ);


  float return_gyroX();
  float return_gyroY();
  float return_gyroZ();

//-----------------------------------------------------------------------------------------------------------------------------
// Pressure readings given by the pressure sensor

  void set_pressure(float pressure);
  float return_pressure();

//-----------------------------------------------------------------------------------------------------------------------------
// PWM values that are pushed to thrusters

  void set_PWM_Side(int* PWM);
  void set_PWM_Back(int* PWM);
  void set_PWM_Bottom(int* PWM);

  // Get PWM readings

  void return_PWM_Side(int* PWM);
  void return_PWM_Back(int* PWM);
  void return_PWM_Bottom(int* PWM);
  
// Call boundary check just before these values are read by the Arduino and given to the thrusters via ESC
  void boundaryCheck(int* PWM);    // IF between 1525 and 1475 make it 1500. If it is more than 1900 make it 1900 (here I conservatively chose 1875 as the max instead of 1900)


  //Shared memory functions

  int attach_r(int key);
  int attach_theta(int key);
  int attach_phi(int key);
};

#endif
