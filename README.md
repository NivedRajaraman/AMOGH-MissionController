# AMOGH - Mission Controller
Mission Controller source and documentation for AUV Amogh. AUV Amogh is an Autonomous Underwater Vehicle designed and developed by undergraduates at IIT-Madras.
This repository houses the source files and documentation for Amogh's Mission Controller.

### What is the Mission Controller?
The Mission Controller behaves as the basic i/o and control system of the AUV. It operates on two layers:
  - System Layer: The Mission Controller is equipped with serial communication protocols and a shared memory library to enable the various blocks of the AUV to intercommunicate - the IP (Image Processor) and microcontroller.
  - Control Layer: The control layer of the AUV is comprised of various control sequences that translate what the IP sees into commands for the microcontroller that directly control the thrusters.

The current microcontroller in use is Arduino Mega 2560 (subject to change)

### How do I use the Mission Controller?
Simply clone this repository or download it into your working directory. You will need boost to compile this library. The project binaries were generated with boost v1.62.

### Tree Structure of the Mission Controller
```
AMOGH  
├───ORIGINAL                Archive
│   ├───Arduino-serial      Contains the library that interfaces between the motherboard and the Arduino
│   ├───IMU-serial          Contains the library that interfaces between the motherboard and the IMU
│   │   └───Sparton
│   └───ESC                 Contains the library that interfaces between the Arduino and ESC boards
│  
└───WORKING_DIRECTORY  
    ├───Important
    ├───MATLAB              Matlab simulations
    ├───src                 Project source code
    └───TEST                Test code to simulate (a very simplistic) environment
```

##### Additional help/documentation about the working of individual modules can be found within the code.
