#ifndef AHRS8_H
#define AHRS8_H

#include "boost_serial.h"
#include "state.hpp"
#include <math.h>

#define GYRO_CONSTANT (105.788429)*(180.0/M_PI)

class AHRS8{

private:
	char *dataPacket;	//form m of magp to m of magp --- dint include the last m
	
	Serial serial;		//baud rate=115200, 	parity=none,	stop bits=one
	
	void loadScripts();
	float getAttitude(std::string attitude);
	float getVariants(std::string variant);
	void parseDataPacket(char* dataPacket);
	
	

public:
	
	float yaw,  pitch, roll;
	float mag[3];
	float acl[3];		//Coordinate 0-X, 1-Y, 2-Z
	float gyro[3];
	
	
	
	AHRS8();		
	~AHRS8();
	bool serialInitialize(std::string serial_port);		 //args - serial port 
	
	void fetchSerialData(STATE* shm);
	
	float getHeading();
	float getPitch();
	float getRoll();
};



#endif
