/*
  HOW DO I USE MAIN.CPP TO RUN MY AUV?
  Easy.
  1) Figure out the sequence in which your mission is going to run - path task -> gate task -> buoy task (for example).
  2) The control objects in the AUV have all their control algorithms tuned, so you only need to call the respective functions
     from each control object (depending on the task) to move the AUV (the thruster control (PWM value updation etc.) is all 
     taken care of internally)

  WHAT IS THE HIERARCHY OF THE CONTROL OBJECTS?
  The basic (default) thruster control class is the mission controller class. The task control objects are extended subclasses
  of the mission controller class (GateController.cpp, MarkerController.cpp etc.). a single instantiation each of all the
  different types of controlling objects is performed.

  HOW DO THE CONTROL OBJECTS WORK TOGETHER?
  All these common control classes share some common static variables.
  when the IP inputs a change of task, the mission controller gives control to a new derived class which needs all the
  information about the current state of the auv. instead of updating the new data into the new object, it is easier for them
  to share static variables (through the parent object - missioncontroller) to access the AUV state.
*/

// steady_clock is a lifesaver =>   auto Time = std::chrono::steady_clock::now();


#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <pthread.h>
#include <string.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions      */ 
#include <errno.h>   /* ERROR Number Definitions           */

#include <chrono>	// for steady_clock

//#include "IMU_AHRS8.h"

#include "state.hpp"
#include "MissionController.hpp"
#include "BasicController.hpp"
#include "LaneController.hpp"
#include "BuoyController.hpp"
#include "GateController.hpp"
#include "MarkerController.hpp"

main()
{

  // Include this line in the final iteration
  // int ArduinoPort = openPortToArduino();

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&      CREATING SHARED MEMORY BLOCK (PERMA-ATTACHED TO MC)      &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
  
  STATE* shmBlock;    // pointer to the shared memory block
  shmBlock = new STATE();



  /////delete this ( for testing )
   int shmid;          // ID of shared memory segment
  key_t key;          // key of the segment
  void* shmad;        // address of shared memory block

  key = 5678;
  if ((shmid = shmget(key, 88, IPC_CREAT | 0666)) < 0) {
    perror("shmget");
    exit(1);
  }
  if ((shmad = (void *)shmat(shmid, NULL, 0)) == (void *) -1) {
    perror("shmat");
    exit(1);
  }
  shmBlock = new (shmad) STATE();
  /////////


  
//ATTACH SHARED MEMORY BLOCK TO THE MISSION CONTROLLER (KEEP IT ATTACHED AT ALL TIMES)

// Defining the key of the shared memory segment
  printf("Shared memory going to be created...\n");
  shmBlock->attach_taskID(500);
  shmBlock->attach_r(1000);
  shmBlock->attach_theta(2000);
  shmBlock->attach_phi(3000);
  printf("Shared memory created...\n");
// Attach the segment to our data space
  printf("Shared memory segment attached to Mission Controller...\n");
  
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&       BEGIN READING FROM THE IMU       &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*
  AHRS8 IMU;

  if(IMU.serialInitialize("/dev/ttyUSB1"))
  {
    printf("IMU port opened successfully...\n");
  }
  else
  {
    printf("IMU port could not be opened - Please check that the IMU is connected properly or that there is no port clash.\n");
    exit(0);
  }
  
  IMU.fetchSerialData(shmBlock);
*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&       CONTROL OBJECT DEFINITIONS       &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/

// OBJECT DECLARATIONS: THESE CONTROL THE STATE PARAMETERS AND OUTPUT THE PWM VALUES THAT DRIVE THE AUV IN THE NEXT TIMESTEP
// Control is given to each object during execution of that particular task
  
  MissionController* MissionC = new MissionController(shmBlock);

  BasicController* BasicC = new BasicController(shmBlock);

  LaneController* LaneC = new LaneController(shmBlock);

  BuoyController* BuoyC = new BuoyController(shmBlock);
  
  GateController* GateC = new GateController(shmBlock);

  MarkerController* MarkerC = new MarkerController(shmBlock);


  printf("Control objects initialized...\n");

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&        EXECUTION        &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/

  // Include the below 2 lines in the final iteration
	
  // shmBlock->set_port(ArduinoPort);
	// startTime = MissionController->global_time();


	// MOVE missioncontroller forwards till the task shows up as pathTask ('P'). 
  // Till then the task shows up as 'N' which stands for No Task (used to evaluate condition)
  // Once the lane is visible, hit the target r = 0 condition
  // 5 second pre-start wait
  for (int i = 0; i >= 0; i--) {
    printf("%d...\r", i);
    fflush(stdout);
    sleep(1);
  }
  printf("Running Tasks Sequentially...\n");
  
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&     TASKS     &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&    depending on the task, different control objects change the state of the AUV  &&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/

  shmBlock->set_r(10);
  shmBlock->set_taskID('p');
  LaneC->Task("Align", "Surge", 10);
  //LaneC->Task("Target");
  BasicC->timedTask("Hover", 2);
  
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&      MEMORY DEALLOCATIONS, CLOSE HANDLES       &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&   perform all memory deallocations and close open handles   &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/

  // Include in final iteration
  // closePortToArduino(ArduinoPort);

  delete BasicC;
  delete LaneC;
  delete MarkerC;
  delete GateC;
  delete BuoyC;

  delete MissionC;
  //delete shmBlock;
  exit(0);
}
